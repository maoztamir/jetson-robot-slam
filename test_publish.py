"""Minimal MQTT5 publish test — mirrors the working sample's approach."""

import json
import threading
import time
from awsiot import mqtt5_client_builder
from awscrt import mqtt5

ENDPOINT = "aw4tyjjeoxel4-ats.iot.us-east-2.amazonaws.com"
CERT = "certs/tele2-jetson.cert.pem"
KEY = "certs/tele2-jetson.private.key"
CLIENT_ID = "basicPubSub"
TOPIC = "robot/tele2-jetson/trajectory"

connected = threading.Event()
stopped = threading.Event()


def on_success(data):
    print(f"Connected: reason={data.connack_packet.reason_code!r}")
    connected.set()

def on_failure(data):
    print(f"Connection failed: {data.exception}")
    connected.set()

def on_stopped(data):
    stopped.set()

def on_disconnect(data):
    rc = data.disconnect_packet.reason_code if data.disconnect_packet else "None"
    print(f"Disconnected: {rc}")


client = mqtt5_client_builder.mtls_from_path(
    endpoint=ENDPOINT,
    cert_filepath=CERT,
    pri_key_filepath=KEY,
    client_id=CLIENT_ID,
    on_lifecycle_connection_success=on_success,
    on_lifecycle_connection_failure=on_failure,
    on_lifecycle_stopped=on_stopped,
    on_lifecycle_disconnection=on_disconnect,
)

print("Starting client...")
client.start()

if not connected.wait(10):
    print("TIMEOUT waiting for connection")
    raise SystemExit(1)

# Publish 3 test messages to the robot trajectory topic
for i in range(3):
    payload = json.dumps({
        "timestamp": round(time.time(), 4),
        "device_id": "tele2-jetson",
        "position": {"x": i * 0.1, "y": 0.0, "z": 0.0},
        "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
        "test": True,
    })

    result = client.publish(mqtt5.PublishPacket(
        topic=TOPIC,
        payload=payload,
        qos=mqtt5.QoS.AT_LEAST_ONCE,
    )).result(10)

    puback = result.puback
    print(f"[{i+1}/3] Published to {TOPIC} -- puback reason={puback.reason_code!r}")
    time.sleep(1)

# Also try the sdk/test/python topic (the one that works in the sample)
print(f"\nPublishing to sdk/test/python for comparison...")
result = client.publish(mqtt5.PublishPacket(
    topic="sdk/test/python",
    payload="hello from test_publish.py",
    qos=mqtt5.QoS.AT_LEAST_ONCE,
)).result(10)
print(f"Published to sdk/test/python -- puback reason={result.puback.reason_code!r}")

print("\nStopping client...")
client.stop()
stopped.wait(5)
print("Done.")
