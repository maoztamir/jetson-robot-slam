"""DynamoDB Stream processor for robot trajectory data.

Triggered by INSERT events on the ``RobotTrajectory`` DynamoDB table.
Each record contains a 6-DoF pose published by the Jetson robot.  This
function:

1. Extracts the device ID and SLAM-frame (x, y) position from the
   stream image.
2. Converts local SLAM metres into WGS-84 latitude/longitude using a
   configurable GPS origin.
3. Calls **AWS Location Service** ``BatchUpdateDevicePosition`` to
   update the tracker, making the position available to the web
   dashboard.

Environment variables (set by CloudFormation)::

    TRACKER_NAME  -- Location Service tracker name
    ORIGIN_LAT    -- GPS latitude of the SLAM origin  (degrees)
    ORIGIN_LON    -- GPS longitude of the SLAM origin (degrees)

DynamoDB stream record ``NewImage`` layout (DynamoDB JSON)::

    {
      "device_id":   {"S": "jetson-robot-01"},
      "timestamp":   {"N": "1234.5678"},
      "position":    {"M": {"x": {"N": "1.23"}, "y": {"N": "-0.45"}, "z": {"N": "0.0"}}},
      "orientation": {"M": {"w": {"N": "0.92"}, "x": {"N": "0.0"}, ...}},
      ...
    }
"""

from __future__ import annotations

import logging
import math
import os
import time
from typing import Any, Dict, List, Optional, Tuple

import boto3
from botocore.exceptions import ClientError

# ---------------------------------------------------------------------------
# Logging -- CloudWatch picks up anything written to stdout/stderr, but
# using the logging module gives us structured level/timestamp for free.
# ---------------------------------------------------------------------------
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# ---------------------------------------------------------------------------
# Configuration from environment
# ---------------------------------------------------------------------------
TRACKER_NAME: str = os.environ.get("TRACKER_NAME", "robot-trajectory-tracker")
ORIGIN_LAT: float = float(os.environ.get("ORIGIN_LAT", "0.0"))
ORIGIN_LON: float = float(os.environ.get("ORIGIN_LON", "0.0"))

# ---------------------------------------------------------------------------
# AWS clients -- reused across warm invocations
# ---------------------------------------------------------------------------
location_client = boto3.client("location")


# ---------------------------------------------------------------------------
# Coordinate conversion
# ---------------------------------------------------------------------------

# WGS-84 constants
_METRES_PER_DEG_LAT: float = 111_320.0


def slam_to_gps(
    x_metres: float,
    y_metres: float,
    origin_lat: float,
    origin_lon: float,
) -> Tuple[float, float]:
    """Convert SLAM-frame metres to WGS-84 (longitude, latitude).

    The SLAM coordinate system is a local East-North-Up (ENU) frame
    anchored at (``origin_lat``, ``origin_lon``):

    * **x** → East  (positive = east of origin)
    * **y** → North (positive = north of origin)

    Longitude degrees are narrower than latitude degrees at latitudes
    away from the equator.  We account for this with the standard
    ``cos(lat)`` correction.

    Args:
        x_metres: SLAM x coordinate (east, in metres).
        y_metres: SLAM y coordinate (north, in metres).
        origin_lat: GPS latitude of the SLAM origin (degrees).
        origin_lon: GPS longitude of the SLAM origin (degrees).

    Returns:
        ``(longitude, latitude)`` in decimal degrees.  The order
        matches the GeoJSON / Location Service convention
        (longitude first).
    """
    # Latitude offset: y metres / (metres per degree of latitude).
    lat = origin_lat + y_metres / _METRES_PER_DEG_LAT

    # Longitude offset: x metres / (metres per degree of longitude).
    # One degree of longitude = 111 320 * cos(latitude) metres.
    metres_per_deg_lon = _METRES_PER_DEG_LAT * math.cos(math.radians(origin_lat))

    # Guard against division by zero at the poles.
    if metres_per_deg_lon > 1.0:
        lon = origin_lon + x_metres / metres_per_deg_lon
    else:
        lon = origin_lon

    return (lon, lat)


# ---------------------------------------------------------------------------
# DynamoDB stream record parsing
# ---------------------------------------------------------------------------

def _extract_pose(image: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Extract device_id, timestamp, and position from a stream NewImage.

    Args:
        image: The ``NewImage`` dict from the DynamoDB stream record
               (DynamoDB JSON with type descriptors).

    Returns:
        A dict with ``device_id``, ``timestamp``, ``x``, ``y``, ``z``
        or ``None`` if required fields are missing.
    """
    try:
        device_id = image["device_id"]["S"]
        timestamp = float(image["timestamp"]["N"])
        pos = image["position"]["M"]
        x = float(pos["x"]["N"])
        y = float(pos["y"]["N"])
        z = float(pos["z"]["N"])
        return {
            "device_id": device_id,
            "timestamp": timestamp,
            "x": x,
            "y": y,
            "z": z,
        }
    except (KeyError, TypeError, ValueError) as exc:
        logger.warning("Skipping malformed record: %s", exc)
        return None


# ---------------------------------------------------------------------------
# Location Service update
# ---------------------------------------------------------------------------

def _update_tracker(
    updates: List[Dict[str, Any]],
) -> int:
    """Push a batch of positions to the Location Service tracker.

    Args:
        updates: List of dicts from ``_extract_pose``.

    Returns:
        Number of positions successfully submitted.
    """
    if not updates:
        return 0

    # Build the Updates list for BatchUpdateDevicePosition.
    # Each entry needs DeviceId, Position [lon, lat], and SampleTime.
    entries = []
    for u in updates:
        lon, lat = slam_to_gps(u["x"], u["y"], ORIGIN_LAT, ORIGIN_LON)

        # SampleTime must be an ISO-8601 string or a datetime.
        # We convert the monotonic SLAM timestamp to a wall-clock
        # approximation (good enough for the tracker).
        sample_time = time.strftime(
            "%Y-%m-%dT%H:%M:%SZ", time.gmtime(u["timestamp"]),
        )

        entries.append({
            "DeviceId": u["device_id"],
            "Position": [lon, lat],
            "SampleTime": sample_time,
        })

    try:
        response = location_client.batch_update_device_position(
            TrackerName=TRACKER_NAME,
            Updates=entries,
        )

        errors = response.get("Errors", [])
        if errors:
            for err in errors:
                logger.error(
                    "Tracker update failed for %s: %s",
                    err.get("DeviceId", "?"),
                    err.get("Error", {}).get("Message", "unknown"),
                )

        succeeded = len(entries) - len(errors)
        logger.info(
            "Tracker updated: %d/%d positions for tracker '%s'",
            succeeded, len(entries), TRACKER_NAME,
        )
        return succeeded

    except ClientError as exc:
        logger.error(
            "BatchUpdateDevicePosition failed: %s",
            exc.response["Error"]["Message"],
        )
        return 0


# ---------------------------------------------------------------------------
# Lambda handler
# ---------------------------------------------------------------------------

def handler(event: Dict[str, Any], context: Any) -> Dict[str, Any]:
    """Lambda entry point -- processes DynamoDB Stream records.

    Args:
        event: DynamoDB Stream event containing a ``Records`` list.
        context: Lambda context (unused, but required by the runtime).

    Returns:
        A dict with ``statusCode`` and ``body`` for diagnostics.
    """
    records = event.get("Records", [])
    logger.info(
        "Received %d record(s) from DynamoDB Stream", len(records),
    )

    poses: List[Dict[str, Any]] = []

    for record in records:
        # Only process INSERT events (the CloudFormation filter should
        # already guarantee this, but belt-and-suspenders).
        if record.get("eventName") != "INSERT":
            continue

        image = record.get("dynamodb", {}).get("NewImage")
        if image is None:
            continue

        pose = _extract_pose(image)
        if pose is not None:
            poses.append(pose)

    if not poses:
        logger.info("No valid poses in this batch -- nothing to do")
        return {"statusCode": 200, "body": "no_poses"}

    succeeded = _update_tracker(poses)

    return {
        "statusCode": 200,
        "body": f"processed={len(poses)},updated={succeeded}",
    }
