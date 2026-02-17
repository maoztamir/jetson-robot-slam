"""AWS IoT Core MQTT publisher for robot telemetry.

Publishes 6-DoF poses and system telemetry to AWS IoT Core using the
``aws-iot-device-sdk-python-v2`` (MQTT5).  Uses the system trust store
by default so no root CA file is needed.  Features automatic
reconnection (handled by the v2 SDK), configurable pose throttling,
and compact JSON payloads (floats rounded to 4 d.p.).

When the SDK is not installed or the connection fails the publisher
operates in **mock mode** — every method succeeds but messages are
only written to the debug log.

Typical usage::

    pub = AWSIoTPublisher(
        endpoint="abc.iot.us-east-1.amazonaws.com",
        thing_name="jetson-robot-01",
        cert_path="certs/certificate.pem.crt",
        key_path="certs/private.pem.key",
    )
    pub.connect()

    # In the SLAM loop:
    pub.publish_pose(pose_4x4, timestamp)

    # Periodically:
    pub.publish_telemetry({"cpu_temp_c": 52, "slam_state": "TRACKING"})

    pub.disconnect()
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from typing import Any, Dict, Optional

import numpy as np

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Quaternion helper
# ---------------------------------------------------------------------------

def rotation_matrix_to_quaternion(R: np.ndarray) -> Dict[str, float]:
    """Convert a 3x3 rotation matrix to a normalised quaternion dict.

    Uses the Shepperd method which is numerically stable for all
    rotation magnitudes.

    Args:
        R: 3x3 orthogonal rotation matrix.

    Returns:
        ``{"w": …, "x": …, "y": …, "z": …}`` with values rounded to
        4 decimal places.
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    # Normalise to unit length.
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm > 0.0:
        w /= norm
        x /= norm
        y /= norm
        z /= norm

    return {
        "w": round(w, 4),
        "x": round(x, 4),
        "y": round(y, 4),
        "z": round(z, 4),
    }


# ---------------------------------------------------------------------------
# Publisher
# ---------------------------------------------------------------------------

class AWSIoTPublisher:
    """MQTT5 publisher for AWS IoT Core with reconnect and throttling.

    Args:
        endpoint:         AWS IoT data endpoint
                          (``*.iot.<region>.amazonaws.com``).
        thing_name:       IoT Thing name — used as the ``device_id`` in
                          payloads and in topic paths.
        cert_path:        Path to the device certificate (PEM).
        key_path:         Path to the device private key (PEM).
        root_ca:          Optional path to a custom root CA certificate.
                          When *None* the system trust store is used.
        client_id:        MQTT client ID.  Defaults to *thing_name* but
                          can be set independently to match the IoT
                          policy (e.g. ``basicPubSub``).
        publish_interval: Only publish every *N*-th pose (1 = every).
        topic_prefix:     MQTT topic root.  Poses are published to
                          ``<prefix>/<thing_name>/trajectory``.
        qos:              MQTT QoS level (0 or 1).
    """

    def __init__(
        self,
        endpoint: str,
        thing_name: str,
        cert_path: str,
        key_path: str,
        *,
        root_ca: Optional[str] = None,
        client_id: Optional[str] = None,
        publish_interval: int = 5,
        topic_prefix: str = "robot",
        qos: int = 1,
    ) -> None:
        self._endpoint = endpoint
        self._thing_name = thing_name
        self._client_id = client_id or thing_name
        self._root_ca = root_ca
        self._cert_path = cert_path
        self._key_path = key_path
        self._publish_interval = max(1, publish_interval)
        self._qos = qos

        self._client: Any = None          # Mqtt5Client or None
        self._connected: bool = False
        self._mock: bool = False

        self._lock = threading.Lock()
        self._connection_event = threading.Event()
        self._pose_counter: int = 0
        self._publish_count: int = 0
        self._error_count: int = 0

        # Topics
        self._topic_trajectory = f"{topic_prefix}/{thing_name}/trajectory"
        self._topic_telemetry = f"{topic_prefix}/{thing_name}/telemetry"

    # ---------------------------------------------------------------- public

    @property
    def is_connected(self) -> bool:
        """``True`` when the MQTT connection is live."""
        return self._connected

    @property
    def publish_count(self) -> int:
        """Total successful publishes since construction."""
        return self._publish_count

    @property
    def error_count(self) -> int:
        """Total failed publish attempts since construction."""
        return self._error_count

    # -- lifecycle --------------------------------------------------------- #

    def connect(self) -> None:
        """Establish the MQTT5 connection to AWS IoT Core.

        On failure (missing SDK, bad certificates, network error) the
        publisher silently falls back to **mock mode** so the SLAM loop
        keeps running.
        """
        try:
            from awsiot import mqtt5_client_builder  # noqa: F811
            from awscrt import mqtt5 as _mqtt5

            def _on_lifecycle_connection_success(lifecycle_connect_success_data):
                self._connected = True
                self._connection_event.set()

            def _on_lifecycle_connection_failure(lifecycle_connection_failure_data):
                connack = getattr(lifecycle_connection_failure_data, "connack_packet", None)
                exc = getattr(lifecycle_connection_failure_data, "exception", None)
                logger.error(
                    "MQTT5 connection failure -- connack=%s  exception=%s",
                    connack, exc,
                )
                self._connection_event.set()

            def _on_lifecycle_stopped(lifecycle_stopped_data):
                self._connected = False

            def _on_lifecycle_disconnection(lifecycle_disconnect_data):
                self._connected = False

            builder_kwargs = dict(
                endpoint=self._endpoint,
                port=8883,
                cert_filepath=self._cert_path,
                pri_key_filepath=self._key_path,
                client_id=self._client_id,
                on_lifecycle_connection_success=_on_lifecycle_connection_success,
                on_lifecycle_connection_failure=_on_lifecycle_connection_failure,
                on_lifecycle_stopped=_on_lifecycle_stopped,
                on_lifecycle_disconnection=_on_lifecycle_disconnection,
            )
            if self._root_ca:
                builder_kwargs["ca_filepath"] = self._root_ca

            client = mqtt5_client_builder.mtls_from_path(**builder_kwargs)
            client.start()

            # Wait for the connection lifecycle event (up to 10 s).
            if not self._connection_event.wait(timeout=10):
                raise TimeoutError("MQTT5 connection timed out after 10 s")

            if not self._connected:
                raise ConnectionError("MQTT5 lifecycle reported connection failure")

            self._client = client
            self._mock = False
            logger.info(
                "Connected to AWS IoT Core at %s as '%s'",
                self._endpoint, self._thing_name,
            )

        except ImportError:
            logger.warning(
                "aws-iot-device-sdk-v2 not installed -- running in mock mode"
            )
            self._mock = True

        except FileNotFoundError as exc:
            logger.error(
                "Certificate file missing: %s -- running in mock mode", exc,
            )
            self._mock = True

        except Exception as exc:
            logger.error(
                "AWS IoT connection failed (%s) -- running in mock mode", exc,
            )
            self._mock = True

    def disconnect(self) -> None:
        """Disconnect from AWS IoT Core."""
        if self._client is not None and self._connected:
            try:
                self._client.stop()
                logger.info("Disconnected from AWS IoT Core")
            except Exception:
                logger.debug("Disconnect error", exc_info=True)
        self._connected = False
        logger.debug(
            "Publisher stopped -- %d published, %d errors",
            self._publish_count, self._error_count,
        )

    # -- publishing -------------------------------------------------------- #

    @staticmethod
    def build_pose_payload(
        pose_matrix: np.ndarray,
        timestamp: float,
        thing_name: str,
    ) -> dict:
        """Build a pose payload dict from a 4x4 pose matrix.

        This is the canonical format used for both MQTT publishing and
        local telemetry file recording.

        Args:
            pose_matrix: 4x4 homogeneous ``T_wc`` (world <- camera).
            timestamp:   Monotonic time of the pose.
            thing_name:  Device / thing identifier.

        Returns:
            Dict with ``timestamp``, ``device_id``, ``position``, and
            ``orientation`` keys.
        """
        position = pose_matrix[:3, 3]
        orientation = rotation_matrix_to_quaternion(pose_matrix[:3, :3])

        return {
            "timestamp": round(float(timestamp), 4),
            "device_id": thing_name,
            "position": {
                "x": round(float(position[0]), 4),
                "y": round(float(position[1]), 4),
                "z": round(float(position[2]), 4),
            },
            "orientation": orientation,
        }

    def publish_pose(
        self,
        pose_matrix: np.ndarray,
        timestamp: float,
    ) -> bool:
        """Publish a 4x4 pose matrix to the trajectory topic.

        The call is **throttled**: only every *publish_interval*-th
        invocation actually transmits.  Other calls return ``False``
        immediately.

        Args:
            pose_matrix: 4x4 homogeneous ``T_wc`` (world <- camera).
            timestamp:   Monotonic time of the pose.

        Returns:
            ``True`` if the message was published (or queued),
            ``False`` if throttled or failed.
        """
        self._pose_counter += 1
        if self._pose_counter % self._publish_interval != 0:
            return False

        payload = self.build_pose_payload(pose_matrix, timestamp, self._thing_name)
        return self._publish(self._topic_trajectory, payload)

    def publish_telemetry(self, data: Dict[str, Any]) -> bool:
        """Publish a free-form telemetry dict.

        ``device_id`` and ``timestamp`` are added automatically if not
        already present.

        Args:
            data: Arbitrary telemetry key/values (cpu_temp, slam_state,
                  battery_pct, fps, etc.).

        Returns:
            ``True`` on success, ``False`` on failure.
        """
        payload = dict(data)
        payload.setdefault("device_id", self._thing_name)
        payload.setdefault("timestamp", round(time.time(), 4))
        return self._publish(self._topic_telemetry, payload)

    # --------------------------------------------------------------- private

    def _publish(self, topic: str, payload: dict) -> bool:
        """Serialise *payload* and publish to *topic*.

        In mock mode the message is logged at DEBUG level and counted
        as a successful publish so counters stay meaningful during
        development.
        """
        body = json.dumps(payload, separators=(",", ":"))

        if self._mock or not self._connected:
            logger.debug("[mock] Published to %s (device=%s)", topic, self._thing_name)
            with self._lock:
                self._publish_count += 1
            return True

        try:
            from awscrt.mqtt5 import PublishPacket, QoS

            qos = QoS.AT_LEAST_ONCE if self._qos == 1 else QoS.AT_MOST_ONCE
            publish_future = self._client.publish(PublishPacket(
                topic=topic,
                payload=body.encode("utf-8"),
                qos=qos,
            ))
            result = publish_future.result(timeout=5)
            puback = getattr(result, "puback", None)
            if puback:
                reason = puback.reason_code
                logger.debug(
                    "Published to %s (device=%s) puback=%s",
                    topic, self._thing_name, repr(reason),
                )
            else:
                logger.debug("Published to %s (device=%s)", topic, self._thing_name)
            with self._lock:
                self._publish_count += 1
            return True

        except Exception as exc:
            with self._lock:
                self._error_count += 1
            logger.warning("Publish to %s failed: %s", topic, exc)
            return False


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.WARNING,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    pub = AWSIoTPublisher(
        endpoint="test.iot.us-east-1.amazonaws.com",
        thing_name="jetson-robot-01",
        cert_path="certs/certificate.pem.crt",
        key_path="certs/private.pem.key",
        publish_interval=3,
    )
    pub.connect()  # will fall back to mock mode

    print(f"Connected: {pub.is_connected}  (mock mode expected)\n")

    # --- Pose publishing (only every 3rd call transmits) ---
    print("Publishing 15 poses (expect 5 actual publishes):\n")
    for i in range(15):
        angle = i * 0.2
        c, s = math.cos(angle), math.sin(angle)

        T = np.eye(4)
        T[0, 0] = c;   T[0, 1] = -s
        T[1, 0] = s;   T[1, 1] = c
        T[0, 3] = 2.0 * c
        T[1, 3] = 2.0 * s

        sent = pub.publish_pose(T, time.monotonic())
        if sent:
            pos = T[:3, 3]
            print(
                f"  #{i + 1:>2}  SENT  "
                f"pos=({pos[0]:+6.3f}, {pos[1]:+6.3f}, {pos[2]:+6.3f})"
            )

    # --- Telemetry publishing ---
    print("\nPublishing telemetry:")
    pub.publish_telemetry({
        "cpu_temp_c": 52.3,
        "gpu_temp_c": 48.1,
        "mem_used_mb": 2100,
        "slam_state": "TRACKING",
        "fps": 28.5,
    })

    pub.disconnect()

    print(f"\nTotal publishes : {pub.publish_count}")
    print(f"Total errors    : {pub.error_count}")
    print(f"Poses submitted : {pub._pose_counter}")
    print(f"Poses sent      : {pub._pose_counter // 3}")
