#!/usr/bin/env python3
"""Subscribe to all robot/# MQTT topics and print incoming messages."""

import json
import sys
import threading
import time
from pathlib import Path

from awsiot import mqtt5_client_builder
from awscrt import mqtt5

# ---------------------------------------------------------------------------
# Config — edit or pass via CLI
# ---------------------------------------------------------------------------
PROJECT_ROOT = Path(__file__).resolve().parent.parent

ENDPOINT    = "aw4tyjjeoxel4-ats.iot.us-east-2.amazonaws.com"
CERT        = str(PROJECT_ROOT / "certs" / "telemetrybox-01.cert.pem")
KEY         = str(PROJECT_ROOT / "certs" / "telemetrybox-01.private.key")
ROOT_CA     = str(PROJECT_ROOT / "certs" / "root-CA.crt")
CLIENT_ID   = "robot-subscriber"
TOPIC_FILTER = "robot/#"          # matches all robot/... topics

# ---------------------------------------------------------------------------
connected   = threading.Event()
# ---------------------------------------------------------------------------


def on_lifecycle_connection_success(event):
    print(f"[connected] negotiated protocol={event.negotiated_settings.server_keep_alive}")
    connected.set()


def on_lifecycle_disconnected(event):
    print(f"[disconnected] reason={event.disconnect_data}")


def on_publish_received(event):
    topic   = event.publish_packet.topic
    payload = event.publish_packet.payload
    try:
        data = json.loads(payload)
        pretty = json.dumps(data, indent=2)
    except Exception:
        pretty = payload.decode("utf-8", errors="replace")

    ts = time.strftime("%H:%M:%S")
    print(f"\n[{ts}] {topic}")
    print(pretty)
    sys.stdout.flush()


def main():
    print(f"Connecting to {ENDPOINT} as '{CLIENT_ID}'")
    print(f"Subscribing to: {TOPIC_FILTER}")
    print("Press Ctrl+C to stop.\n")

    client = mqtt5_client_builder.mtls_from_path(
        endpoint=ENDPOINT,
        cert_filepath=CERT,
        pri_key_filepath=KEY,
        ca_filepath=ROOT_CA,
        client_id=CLIENT_ID,
        on_publish_received=on_publish_received,
        on_lifecycle_connection_success=on_lifecycle_connection_success,
        on_lifecycle_disconnected=on_lifecycle_disconnected,
    )

    client.start()
    if not connected.wait(timeout=10):
        print("ERROR: connection timed out")
        sys.exit(1)

    subscribe_future = client.subscribe(
        subscribe_packet=mqtt5.SubscribePacket(
            subscriptions=[
                mqtt5.Subscription(topic_filter=TOPIC_FILTER, qos=mqtt5.QoS.AT_LEAST_ONCE)
            ]
        )
    )
    subscribe_future.result(timeout=5)
    print(f"Subscribed to {TOPIC_FILTER} — waiting for messages...\n")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        client.stop()


if __name__ == "__main__":
    main()
