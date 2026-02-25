"""Minimal MQTT5 publish test.

Uses the shared AWSIoTPublisher helper, which gracefully falls back to
mock mode when the AWS IoT SDK is not installed.
"""

import json
import logging
import time

from src.cloud.aws_iot_publisher import AWSIoTPublisher

ENDPOINT = "aw4tyjjeoxel4-ats.iot.us-east-2.amazonaws.com"
CERT = "certs/tele2-jetson.cert.pem"
KEY = "certs/tele2-jetson.private.key"
CLIENT_ID = "basicPubSub"
THING_NAME = "tele2-jetson"
TOPIC = f"robot/{THING_NAME}/trajectory"


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    publisher = AWSIoTPublisher(
        endpoint=ENDPOINT,
        thing_name=THING_NAME,
        cert_path=CERT,
        key_path=KEY,
        client_id=CLIENT_ID,
        publish_interval=1,
    )

    print("Starting publisher...")
    publisher.connect()
    if publisher.is_connected:
        print("Connected to AWS IoT Core.")
    else:
        print("Running in mock mode (SDK missing or connection failed).")

    # Publish 3 test messages to the robot trajectory topic
    for i in range(3):
        payload = {
            "timestamp": round(time.time(), 4),
            "device_id": THING_NAME,
            "position": {"x": i * 0.1, "y": 0.0, "z": 0.0},
            "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            "test": True,
        }

        ok = publisher._publish(TOPIC, payload)
        status = "OK" if ok else "FAILED"
        print(f"[{i + 1}/3] Published to {TOPIC} -- {status}")
        time.sleep(1)

    # Also try the sdk/test/python topic (the one that works in the sample)
    print("\nPublishing to sdk/test/python for comparison...")
    ok = publisher._publish(
        "sdk/test/python",
        {"message": "hello from test_publish.py"},
    )
    print(f"Published to sdk/test/python -- {'OK' if ok else 'FAILED'}")

    print("\nStopping publisher...")
    publisher.disconnect()
    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
