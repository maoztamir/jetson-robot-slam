#!/usr/bin/env python3
"""Local HTTP server for the robot trajectory dashboard.

Serves the web/ dashboard files and provides REST API endpoints that
read from local telemetry files, so the dashboard works without AWS.

Usage::

    python scripts/local_server.py
    python scripts/local_server.py --port 9090
    python scripts/local_server.py --telemetry-dir /tmp/telemetry

Endpoints:
    GET /api/trajectory?device_id=X&since=N  — trajectory poses (JSON array)
    GET /api/status                          — latest sensor status (JSON)
    GET /*                                   — static files from web/
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from functools import partial
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from urllib.parse import parse_qs, urlparse

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
_DEFAULT_TELEMETRY_DIR = _PROJECT_ROOT / "telemetry"
_DEFAULT_WEB_DIR = _PROJECT_ROOT / "web"


class DashboardHandler(SimpleHTTPRequestHandler):
    """Serves static files from web/ and handles /api/* routes."""

    def __init__(self, *args, telemetry_dir: Path, **kwargs):
        self.telemetry_dir = telemetry_dir
        super().__init__(*args, **kwargs)

    # ------------------------------------------------------------------
    # Routing
    # ------------------------------------------------------------------

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == "/api/trajectory":
            self._handle_trajectory(parse_qs(parsed.query))
        elif path == "/api/status":
            self._handle_status()
        else:
            super().do_GET()

    # ------------------------------------------------------------------
    # API: /api/trajectory
    # ------------------------------------------------------------------

    def _handle_trajectory(self, params):
        device_id = params.get("device_id", [None])[0]
        since = float(params.get("since", [0])[0])

        telemetry_file = self.telemetry_dir / "telemetry.jsonl"
        if not telemetry_file.exists():
            self._json_response([])
            return

        poses = []
        try:
            with open(telemetry_file, "r") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        record = json.loads(line)
                    except json.JSONDecodeError:
                        continue

                    # Filter by device_id if specified
                    if device_id and record.get("device_id") != device_id:
                        continue

                    # Filter by timestamp
                    ts = record.get("timestamp", 0)
                    if ts > since:
                        poses.append(record)
        except OSError:
            pass

        self._json_response(poses)

    # ------------------------------------------------------------------
    # API: /api/status
    # ------------------------------------------------------------------

    def _handle_status(self):
        status_file = self.telemetry_dir / "sensor_status.json"
        if not status_file.exists():
            self._json_response({})
            return

        try:
            with open(status_file, "r") as f:
                data = json.load(f)
            self._json_response(data)
        except (OSError, json.JSONDecodeError):
            self._json_response({})

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _json_response(self, data, status=200):
        body = json.dumps(data).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        # Quieter logging: skip static-file requests
        if args and isinstance(args[0], str) and args[0].startswith("GET /api/"):
            super().log_message(format, *args)


def main():
    parser = argparse.ArgumentParser(description="Local dashboard server")
    parser.add_argument(
        "--port", type=int, default=8080,
        help="Port to listen on (default: 8080)",
    )
    parser.add_argument(
        "--telemetry-dir", type=str,
        default=str(_DEFAULT_TELEMETRY_DIR),
        help="Directory containing telemetry.jsonl and sensor_status.json",
    )
    args = parser.parse_args()

    telemetry_dir = Path(args.telemetry_dir)
    web_dir = _DEFAULT_WEB_DIR

    if not web_dir.is_dir():
        print(f"Error: web directory not found at {web_dir}", file=sys.stderr)
        sys.exit(1)

    # Serve static files from web/
    os.chdir(web_dir)

    handler = partial(DashboardHandler, telemetry_dir=telemetry_dir)
    server = HTTPServer(("0.0.0.0", args.port), handler)

    print(f"Serving dashboard at http://0.0.0.0:{args.port}")
    print(f"Reading telemetry from {telemetry_dir}")
    print("Press Ctrl+C to stop")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down.")
        server.server_close()


if __name__ == "__main__":
    main()
