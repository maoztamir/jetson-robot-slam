"""Tests for the process_trajectory Lambda function."""

import importlib
import math
import sys
from unittest.mock import MagicMock, patch

import pytest

# ``aws/lambda/`` can't be imported as ``aws.lambda`` because ``lambda``
# is a Python keyword.  We load the module via importlib instead.


def _load_module():
    """Import aws/lambda/process_trajectory.py as a module."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "process_trajectory",
        "aws/lambda/process_trajectory.py",
    )
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# We need to mock boto3.client before importing the module, because the
# module calls ``boto3.client("location")`` at import time.
_mock_location = MagicMock()

with patch("boto3.client", return_value=_mock_location):
    _mod = _load_module()

slam_to_gps = _mod.slam_to_gps
_extract_pose = _mod._extract_pose
handler = _mod.handler


# ---------------------------------------------------------------------------
# slam_to_gps
# ---------------------------------------------------------------------------

class TestSlamToGps:
    def test_origin_returns_origin(self):
        lon, lat = slam_to_gps(0.0, 0.0, 32.0, 34.0)
        assert lon == pytest.approx(34.0)
        assert lat == pytest.approx(32.0)

    def test_north_offset(self):
        # 111320 m north ≈ 1 degree of latitude
        lon, lat = slam_to_gps(0.0, 111_320.0, 0.0, 0.0)
        assert lat == pytest.approx(1.0, abs=0.001)
        assert lon == pytest.approx(0.0)

    def test_east_offset_at_equator(self):
        # At equator, 1 deg lon ≈ 111320 m
        lon, lat = slam_to_gps(111_320.0, 0.0, 0.0, 0.0)
        assert lon == pytest.approx(1.0, abs=0.001)
        assert lat == pytest.approx(0.0)

    def test_east_offset_at_high_latitude(self):
        # At 60°N, 1 deg lon ≈ 111320 * cos(60°) ≈ 55660 m
        lon, lat = slam_to_gps(55_660.0, 0.0, 60.0, 0.0)
        assert lon == pytest.approx(1.0, abs=0.01)


# ---------------------------------------------------------------------------
# _extract_pose
# ---------------------------------------------------------------------------

class TestExtractPose:
    def test_valid_record(self):
        image = {
            "device_id": {"S": "jetson-01"},
            "timestamp": {"N": "1234.5678"},
            "position": {"M": {
                "x": {"N": "1.23"},
                "y": {"N": "-0.45"},
                "z": {"N": "0.0"},
            }},
        }
        result = _extract_pose(image)
        assert result is not None
        assert result["device_id"] == "jetson-01"
        assert result["timestamp"] == pytest.approx(1234.5678)
        assert result["x"] == pytest.approx(1.23)
        assert result["y"] == pytest.approx(-0.45)
        assert result["z"] == pytest.approx(0.0)

    def test_missing_device_id(self):
        image = {
            "timestamp": {"N": "1234.5678"},
            "position": {"M": {"x": {"N": "0"}, "y": {"N": "0"}, "z": {"N": "0"}}},
        }
        assert _extract_pose(image) is None

    def test_missing_position(self):
        image = {
            "device_id": {"S": "jetson-01"},
            "timestamp": {"N": "1234.5678"},
        }
        assert _extract_pose(image) is None

    def test_malformed_number(self):
        image = {
            "device_id": {"S": "jetson-01"},
            "timestamp": {"N": "not-a-number"},
            "position": {"M": {"x": {"N": "0"}, "y": {"N": "0"}, "z": {"N": "0"}}},
        }
        assert _extract_pose(image) is None


# ---------------------------------------------------------------------------
# handler (end-to-end with mocked Location Service)
# ---------------------------------------------------------------------------

class TestHandler:
    def _make_event(self, device_id="jetson-01", x="1.0", y="2.0"):
        return {
            "Records": [{
                "eventName": "INSERT",
                "dynamodb": {
                    "NewImage": {
                        "device_id": {"S": device_id},
                        "timestamp": {"N": "1000.0"},
                        "position": {"M": {
                            "x": {"N": x},
                            "y": {"N": y},
                            "z": {"N": "0.0"},
                        }},
                        "orientation": {"M": {
                            "w": {"N": "1.0"},
                            "x": {"N": "0.0"},
                            "y": {"N": "0.0"},
                            "z": {"N": "0.0"},
                        }},
                    },
                },
            }],
        }

    def test_handler_processes_insert(self):
        _mock_location.reset_mock()
        _mock_location.batch_update_device_position.return_value = {"Errors": []}

        result = handler(self._make_event(), None)

        assert result["statusCode"] == 200
        assert "processed=1" in result["body"]
        _mock_location.batch_update_device_position.assert_called_once()

    def test_handler_skips_modify_events(self):
        _mock_location.reset_mock()
        event = self._make_event()
        event["Records"][0]["eventName"] = "MODIFY"

        result = handler(event, None)

        assert result["body"] == "no_poses"
        _mock_location.batch_update_device_position.assert_not_called()

    def test_handler_empty_records(self):
        _mock_location.reset_mock()
        result = handler({"Records": []}, None)
        assert result["statusCode"] == 200
        assert result["body"] == "no_poses"
