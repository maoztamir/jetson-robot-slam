# Testing Guide

Local tests for verifying the robot SLAM pipeline before deploying to AWS.
All tests run offline — no AWS credentials or live connections required.

## Prerequisites

Activate the conda environment and install test dependencies:

```bash
conda activate jetson-robot-slam
pip install pytest boto3
```

## Running All Tests

From the project root:

```bash
pytest tests/ -v
```

Expected output: **16 tests passed**.

## What Each Test Covers

### `test_sensor_status.py` — IoT Publisher (Sensor Status)

Tests `AWSIoTPublisher.publish_sensor_status()` in mock mode (no AWS SDK needed).

| Test | What it verifies |
|------|-----------------|
| `test_returns_true_in_mock_mode` | Publishing succeeds and returns `True` when running without AWS |
| `test_increments_publish_count` | The `publish_count` counter goes up by 1 after each publish |
| `test_publishes_to_status_topic` | The MQTT topic is `robot/<thing_name>/status` |
| `test_payload_structure` | Payload contains `device_id` (str), `timestamp` (float), and `sensors` (dict) |
| `test_multiple_publishes_increment_counter` | Counter stays accurate across multiple publishes |

### `test_lambda.py` — Trajectory Processing Lambda

Tests the `aws/lambda/process_trajectory.py` Lambda function with mocked AWS services.

**Coordinate conversion (`slam_to_gps`):**

| Test | What it verifies |
|------|-----------------|
| `test_origin_returns_origin` | Zero offset returns the origin lat/lon unchanged |
| `test_north_offset` | 111,320 m north = +1 degree latitude |
| `test_east_offset_at_equator` | 111,320 m east at equator = +1 degree longitude |
| `test_east_offset_at_high_latitude` | Longitude degrees shrink by `cos(lat)` at 60°N |

**DynamoDB stream parsing (`_extract_pose`):**

| Test | What it verifies |
|------|-----------------|
| `test_valid_record` | Correctly extracts device_id, timestamp, x, y, z from DynamoDB JSON |
| `test_missing_device_id` | Returns `None` for records missing `device_id` |
| `test_missing_position` | Returns `None` for records missing `position` |
| `test_malformed_number` | Returns `None` when numeric fields contain invalid strings |

**Lambda handler (end-to-end):**

| Test | What it verifies |
|------|-----------------|
| `test_handler_processes_insert` | INSERT events are processed and sent to Location Service tracker |
| `test_handler_skips_modify_events` | MODIFY events are ignored (only INSERTs matter) |
| `test_handler_empty_records` | Empty event returns 200 with `no_poses` body |

## Additional Checks

### Shell script syntax

Verify the deploy scripts have no syntax errors:

```bash
bash -n scripts/deploy_lambda.sh
bash -n scripts/deploy_web.sh
```

No output means success.

### Publisher self-test

Run the IoT publisher module directly to see mock-mode output:

```bash
python -m src.cloud.aws_iot_publisher
```

Expected: 5 pose publishes, 1 telemetry publish, 0 errors.

### Web dashboard (manual)

Serve the dashboard locally:

```bash
cd web && python -m http.server 8080
```

Open `http://localhost:8080` and verify:

- Dark-themed dashboard renders
- Map loads tiles from OpenStreetMap
- Console shows a Cognito error (expected — no AWS backend locally)
