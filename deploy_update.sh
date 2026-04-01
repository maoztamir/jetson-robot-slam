#!/bin/bash
# Deploy updated files to their destination directories.
# Run from the jetson-robot-slam root directory.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Deploying updated files..."

cp "$SCRIPT_DIR/main.py"                 "$SCRIPT_DIR/src/main.py"
cp "$SCRIPT_DIR/aws_iot_publisher.py"    "$SCRIPT_DIR/src/cloud/aws_iot_publisher.py"
cp "$SCRIPT_DIR/gps_reader.py"           "$SCRIPT_DIR/src/sensors/gps_reader.py"
cp "$SCRIPT_DIR/camera_imu_handler.py"   "$SCRIPT_DIR/src/sensors/camera_imu_handler.py"

echo "Done. Files deployed:"
echo "  src/main.py"
echo "  src/cloud/aws_iot_publisher.py"
echo "  src/sensors/gps_reader.py"
echo "  src/sensors/camera_imu_handler.py"
