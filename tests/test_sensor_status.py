"""Tests for AWSIoTPublisher.publish_sensor_status() in mock mode."""

import json
import time
from unittest.mock import patch

from src.cloud.aws_iot_publisher import AWSIoTPublisher


def _make_mock_publisher(thing_name="jetson-test-01"):
    """Create a publisher in mock mode (no AWS connection)."""
    pub = AWSIoTPublisher(
        endpoint="test.iot.us-east-1.amazonaws.com",
        thing_name=thing_name,
        cert_path="fake/cert.pem",
        key_path="fake/key.pem",
    )
    pub.connect()  # falls back to mock mode (no SDK installed)
    return pub


class TestPublishSensorStatus:
    def test_returns_true_in_mock_mode(self):
        pub = _make_mock_publisher()
        sensors = {"camera": "ok", "imu": "ok", "slam": "ok", "gps": "n/a"}
        assert pub.publish_sensor_status(sensors) is True

    def test_increments_publish_count(self):
        pub = _make_mock_publisher()
        before = pub.publish_count
        pub.publish_sensor_status({"camera": "ok"})
        assert pub.publish_count == before + 1

    def test_publishes_to_status_topic(self):
        pub = _make_mock_publisher(thing_name="robot-42")
        assert pub._topic_status == "robot/robot-42/status"

    def test_payload_structure(self):
        pub = _make_mock_publisher(thing_name="jetson-01")
        sensors = {"camera": "ok", "imu": "error", "slam": "mock", "gps": "n/a"}

        captured = {}

        original_publish = pub._publish

        def spy_publish(topic, payload):
            captured.update(payload)
            return original_publish(topic, payload)

        pub._publish = spy_publish
        pub.publish_sensor_status(sensors)

        assert captured["device_id"] == "jetson-01"
        assert "timestamp" in captured
        assert isinstance(captured["timestamp"], float)
        assert captured["sensors"] == sensors

    def test_multiple_publishes_increment_counter(self):
        pub = _make_mock_publisher()
        for _ in range(5):
            pub.publish_sensor_status({"camera": "ok"})
        assert pub.publish_count == 5
