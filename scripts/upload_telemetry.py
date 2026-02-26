#!/usr/bin/env python3
"""Upload locally-saved telemetry JSONL files to DynamoDB.

Usage::

    # Upload all pending files in the telemetry/ directory:
    python scripts/upload_telemetry.py

    # Upload a specific file:
    python scripts/upload_telemetry.py telemetry/jetson-robot-01_2024-01-15T10-30-00.jsonl

    # Dry-run: show what would be uploaded without writing to DynamoDB:
    python scripts/upload_telemetry.py --dry-run

After a successful upload the source file is renamed with a ``.uploaded``
suffix so it won't be processed again on the next run.

The JSONL files are written by ``src/main.py`` in the same format that is
published over MQTT.  Each line is a JSON object with at least:

    {"timestamp": 1234.5, "device_id": "jetson-robot-01", ...}

DynamoDB primary key: ``device_id`` (hash) + ``timestamp`` (range).
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

import boto3
from boto3.dynamodb.types import TypeSerializer
from botocore.exceptions import ClientError

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
REGION = "us-east-2"
TABLE_NAME = "RobotTrajectory"
TTL_DAYS = 90  # matches CloudFormation default

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
_TELEMETRY_DIR = _PROJECT_ROOT / "telemetry"

logging.basicConfig(
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
    level=logging.INFO,
)
logger = logging.getLogger("upload_telemetry")

_serializer = TypeSerializer()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _to_dynamo_item(record: Dict[str, Any]) -> Dict[str, Any]:
    """Convert a plain-Python telemetry record to DynamoDB item format.

    Adds ``ingested_at`` (current epoch) and ``ttl`` (epoch + TTL_DAYS).
    """
    now = time.time()
    record.setdefault("ingested_at", round(now, 4))
    record["ttl"] = int(now + TTL_DAYS * 86400)
    return {k: _serializer.serialize(v) for k, v in record.items()}


def _upload_file(
    table,
    path: Path,
    dry_run: bool = False,
) -> int:
    """Upload all records from one JSONL file.

    Args:
        table:   boto3 DynamoDB Table resource.
        path:    Path to the ``.jsonl`` file.
        dry_run: If True, parse and count records but do not write.

    Returns:
        Number of records successfully written (0 for dry-run).
    """
    records: List[Dict[str, Any]] = []
    skipped = 0

    with open(path) as fh:
        for lineno, raw in enumerate(fh, 1):
            raw = raw.strip()
            if not raw:
                continue
            try:
                records.append(json.loads(raw))
            except json.JSONDecodeError as exc:
                logger.warning("%s line %d: JSON parse error: %s", path.name, lineno, exc)
                skipped += 1

    if not records:
        logger.info("%s: empty — nothing to upload", path.name)
        return 0

    logger.info(
        "%s: %d record(s) to upload%s",
        path.name, len(records),
        " (dry-run)" if dry_run else "",
    )
    if skipped:
        logger.warning("%s: %d line(s) skipped due to parse errors", path.name, skipped)

    if dry_run:
        return 0

    # Batch-write in chunks of 25 (DynamoDB limit).
    written = 0
    errors = 0
    dynamodb_client = boto3.client("dynamodb", region_name=REGION)

    for batch_start in range(0, len(records), 25):
        chunk = records[batch_start : batch_start + 25]
        request_items = {
            TABLE_NAME: [
                {"PutRequest": {"Item": _to_dynamo_item(dict(r))}}
                for r in chunk
            ]
        }
        try:
            resp = dynamodb_client.batch_write_item(RequestItems=request_items)
            unprocessed = resp.get("UnprocessedItems", {}).get(TABLE_NAME, [])
            batch_written = len(chunk) - len(unprocessed)
            written += batch_written
            if unprocessed:
                logger.warning(
                    "%s: %d item(s) unprocessed in this batch",
                    path.name, len(unprocessed),
                )
                errors += len(unprocessed)
        except ClientError as exc:
            logger.error(
                "%s: batch_write_item failed: %s",
                path.name, exc.response["Error"]["Message"],
            )
            errors += len(chunk)

    logger.info(
        "%s: uploaded %d/%d record(s)%s",
        path.name, written, len(records),
        f"  ({errors} errors)" if errors else "",
    )
    return written


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument(
        "files",
        nargs="*",
        help="JSONL files to upload. Defaults to all *.jsonl in telemetry/.",
    )
    p.add_argument(
        "--dry-run", action="store_true",
        help="Parse files and report counts without writing to DynamoDB.",
    )
    p.add_argument(
        "--no-rename", action="store_true",
        help="Do not rename files to .uploaded after successful upload.",
    )
    p.add_argument(
        "--region", default=REGION,
        help=f"AWS region (default: {REGION}).",
    )
    p.add_argument(
        "--table", default=TABLE_NAME,
        help=f"DynamoDB table name (default: {TABLE_NAME}).",
    )
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    # Resolve file list.
    if args.files:
        paths = [Path(f) for f in args.files]
    else:
        paths = sorted(_TELEMETRY_DIR.glob("*.jsonl"))
        if not paths:
            logger.info("No *.jsonl files found in %s", _TELEMETRY_DIR)
            return

    dynamodb = boto3.resource("dynamodb", region_name=args.region)
    table = dynamodb.Table(args.table)

    total_written = 0
    total_files = 0

    for path in paths:
        if not path.is_file():
            logger.warning("Not found: %s", path)
            continue
        written = _upload_file(table, path, dry_run=args.dry_run)
        total_written += written
        total_files += 1
        if written > 0 and not args.dry_run and not args.no_rename:
            renamed = path.with_suffix(".jsonl.uploaded")
            path.rename(renamed)
            logger.info("Renamed to %s", renamed.name)

    logger.info(
        "Done: %d file(s) processed, %d record(s) uploaded%s",
        total_files, total_written,
        " (dry-run)" if args.dry_run else "",
    )


if __name__ == "__main__":
    main()
