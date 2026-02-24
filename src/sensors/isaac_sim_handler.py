"""Isaac Sim stereo-camera and IMU handler (ROS 2 bridge).

Drop-in replacement for ``CameraIMUHandler`` that reads stereo image pairs
and IMU data from an NVIDIA Isaac Sim robot via the ROS 2 bridge.

The public interface — ``start``, ``stop``, ``get_stereo_frame``,
``get_latest_imu``, ``frame_queue``, ``imu_queue``, ``is_running`` — is
**identical** to ``CameraIMUHandler`` so that ``main.py`` needs only a
single ``--sim`` flag to switch between hardware and simulation.

Confirmed topic layout (192.168.2.33)
--------------------------------------
    /imu                   sensor_msgs/Imu
    /left/image_raw        sensor_msgs/Image
    /left/camera_info      sensor_msgs/CameraInfo
    /right/image_raw       sensor_msgs/Image
    /right/camera_info     sensor_msgs/CameraInfo

Cross-machine ROS 2 networking
--------------------------------
ROS 2 uses DDS multicast for peer discovery.  If the simulation host
(192.168.2.33) and this machine are on the same LAN subnet, set matching
domain IDs on both and multicast should work automatically:

    export ROS_DOMAIN_ID=0          # same value on both machines
    source /opt/ros/humble/setup.bash

If multicast is blocked (common in corporate networks or across VLANs),
use a CycloneDDS unicast peer file.  Create ~/cyclone_peers.xml:

    <?xml version="1.0" encoding="UTF-8"?>
    <CycloneDDS>
      <Domain>
        <Discovery>
          <Peers>
            <Peer Address="192.168.2.33"/>
          </Peers>
        </Discovery>
      </Domain>
    </CycloneDDS>

Then export before running:

    export CYCLONEDDS_URI=file://$HOME/cyclone_peers.xml

Verify connectivity:

    ros2 topic list               # must show /left/image_raw etc.
    ros2 topic hz /left/image_raw # must print a frequency

Unit conventions
-----------------
``sensor_msgs/Imu`` uses m/s² (linear_acceleration) and rad/s
(angular_velocity).  ORB-SLAM3 expects **g** and **deg/s**.
Conversion is applied on every received message.

Camera intrinsics auto-detection
----------------------------------
This handler subscribes to ``/left/camera_info`` and ``/right/camera_info``.
Once both are received, ``get_camera_intrinsics()`` returns a dict ready to
paste into ``config/stereo_imu_settings.yaml``, saving the manual calibration
step when using simulation.

Dependencies
-------------
    source /opt/ros/humble/setup.bash   # provides rclpy, sensor_msgs
    pip install opencv-python numpy
    # Optional (better image format support):
    sudo apt install ros-humble-cv-bridge
"""

from __future__ import annotations

import logging
import math
import threading
import time
from queue import Empty, Full, Queue
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Physical constants
# ---------------------------------------------------------------------------

_G_MS2: float = 9.80665
_RAD_TO_DEG: float = 180.0 / math.pi

# ---------------------------------------------------------------------------
# Type aliases  (identical to camera_imu_handler.py)
# ---------------------------------------------------------------------------

StereoFrame = Tuple[np.ndarray, np.ndarray, float]
"""``(left_bgr, right_bgr, monotonic_timestamp)``"""

IMUSample = Dict[str, float]
"""Keys: timestamp, accel_x/y/z (g), gyro_x/y/z (deg/s)."""


# ---------------------------------------------------------------------------
# Image conversion
# ---------------------------------------------------------------------------

def _ros_image_to_bgr(msg: Any) -> np.ndarray:
    """Convert ``sensor_msgs/Image`` to a BGR numpy array.

    Tries cv_bridge first (handles all encodings); falls back to a manual
    conversion for the formats Isaac Sim typically publishes (rgb8, bgr8,
    rgba8, mono8).
    """
    try:
        from cv_bridge import CvBridge
        if not hasattr(_ros_image_to_bgr, "_bridge"):
            _ros_image_to_bgr._bridge = CvBridge()  # type: ignore[attr-defined]
        return _ros_image_to_bgr._bridge.imgmsg_to_cv2(  # type: ignore[attr-defined]
            msg, desired_encoding="bgr8"
        )
    except ImportError:
        pass

    enc = msg.encoding.lower()
    arr = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    if enc in ("rgb8", "rgb"):
        return cv2.cvtColor(arr.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
    if enc in ("bgr8", "bgr"):
        return arr.reshape(msg.height, msg.width, 3)
    if enc in ("rgba8", "rgba"):
        return cv2.cvtColor(arr.reshape(msg.height, msg.width, 4), cv2.COLOR_RGBA2BGR)
    if enc in ("bgra8", "bgra"):
        return cv2.cvtColor(arr.reshape(msg.height, msg.width, 4), cv2.COLOR_BGRA2BGR)
    if enc in ("mono8", "8uc1"):
        return cv2.cvtColor(arr.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
    if enc in ("mono16", "16uc1"):
        gray16 = arr.view(np.uint16).reshape(msg.height, msg.width)
        return cv2.cvtColor((gray16 >> 8).astype(np.uint8), cv2.COLOR_GRAY2BGR)

    raise ValueError(f"Unsupported image encoding: {msg.encoding!r}")


# ---------------------------------------------------------------------------
# Timestamp alignment
# ---------------------------------------------------------------------------

def _ros_stamp_to_monotonic(header: Any) -> float:
    """Map a ROS 2 header stamp to ``time.monotonic()`` seconds.

    The offset between ROS wall time and the local monotonic clock is
    computed once on the first call and cached for the process lifetime.
    """
    ros_sec = header.stamp.sec + header.stamp.nanosec * 1e-9
    if _ros_stamp_to_monotonic._offset is None:  # type: ignore[attr-defined]
        _ros_stamp_to_monotonic._offset = time.monotonic() - time.time()  # type: ignore[attr-defined]
    return ros_sec + _ros_stamp_to_monotonic._offset  # type: ignore[attr-defined]


_ros_stamp_to_monotonic._offset = None  # type: ignore[attr-defined]


# ---------------------------------------------------------------------------
# CameraInfo → ORB-SLAM3 intrinsics
# ---------------------------------------------------------------------------

def _parse_camera_info(msg: Any) -> Dict[str, Any]:
    """Extract ORB-SLAM3-compatible intrinsics from a CameraInfo message.

    Args:
        msg: ``sensor_msgs.msg.CameraInfo``

    Returns:
        Dict with keys: fx, fy, cx, cy, k1, k2, p1, p2, width, height,
        and raw_K (3x3 numpy), raw_D (distortion array), raw_P (3x4 numpy).
    """
    K = np.array(msg.k).reshape(3, 3)   # intrinsic matrix
    D = list(msg.d)                       # distortion coefficients
    P = np.array(msg.p).reshape(3, 4)   # projection matrix

    return {
        "fx":     float(K[0, 0]),
        "fy":     float(K[1, 1]),
        "cx":     float(K[0, 2]),
        "cy":     float(K[1, 2]),
        "k1":     float(D[0]) if len(D) > 0 else 0.0,
        "k2":     float(D[1]) if len(D) > 1 else 0.0,
        "p1":     float(D[2]) if len(D) > 2 else 0.0,
        "p2":     float(D[3]) if len(D) > 3 else 0.0,
        "width":  int(msg.width),
        "height": int(msg.height),
        "raw_K":  K,
        "raw_D":  D,
        "raw_P":  P,
    }


# ---------------------------------------------------------------------------
# Main handler
# ---------------------------------------------------------------------------

class IsaacSimCameraIMUHandler:
    """Stereo camera + IMU handler reading from Isaac Sim via ROS 2.

    Subscribes to five topics on the simulation host and exposes the
    same queue/method interface as ``CameraIMUHandler``.

    Args:
        left_topic:         ROS 2 image topic for the left camera.
        right_topic:        ROS 2 image topic for the right camera.
        imu_topic:          ROS 2 IMU topic.
        left_info_topic:    ROS 2 CameraInfo topic for the left camera.
        right_info_topic:   ROS 2 CameraInfo topic for the right camera.
        width:              Resize output to this width (0 = no resize).
        height:             Resize output to this height (0 = no resize).
        sync_threshold:     Max timestamp gap (s) for a valid stereo pair.
        frame_queue_size:   Max buffered stereo pairs.
        imu_queue_size:     Max buffered IMU samples.
        node_name:          Name of the internal rclpy node.
    """

    # Satisfies ``camera._imu_reader._mock`` check in main.py
    class _FakeIMUReader:
        _mock: bool = False   # sim IMU is not a software mock

    def __init__(
        self,
        left_topic:        str = "/left/image_raw",
        right_topic:       str = "/right/image_raw",
        imu_topic:         str = "/imu",
        left_info_topic:   str = "/left/camera_info",
        right_info_topic:  str = "/right/camera_info",
        width:             int = 640,
        height:            int = 480,
        sync_threshold:    float = 0.02,
        frame_queue_size:  int = 2,
        imu_queue_size:    int = 50,
        node_name:         str = "jetson_slam_bridge",
    ) -> None:
        self._left_topic       = left_topic
        self._right_topic      = right_topic
        self._imu_topic        = imu_topic
        self._left_info_topic  = left_info_topic
        self._right_info_topic = right_info_topic
        self._width            = width
        self._height           = height
        self._sync_threshold   = sync_threshold
        self._node_name        = node_name

        # Public queues
        self._frame_q: Queue[StereoFrame] = Queue(maxsize=frame_queue_size)
        self._imu_q:   Queue[IMUSample]   = Queue(maxsize=imu_queue_size)

        # Latest-value cache (lock-protected)
        self._lock = threading.Lock()
        self._latest_frame: Optional[StereoFrame] = None
        self._latest_imu:   Optional[IMUSample]   = None

        # Stereo sync buffers
        self._pending_left:  List[Tuple[float, np.ndarray]] = []
        self._pending_right: List[Tuple[float, np.ndarray]] = []
        self._pending_lock = threading.Lock()

        # Camera intrinsics (populated when camera_info arrives)
        self._left_info:  Optional[Dict[str, Any]] = None
        self._right_info: Optional[Dict[str, Any]] = None
        self._info_lock = threading.Lock()
        self._info_logged = False   # log intrinsics only once

        # Satisfies main.py private-attribute check
        self._imu_reader = self._FakeIMUReader()

        self._running = False
        self._node: Any = None
        self._ros_thread: Optional[threading.Thread] = None

    # =========================================================================
    # Public interface  (identical to CameraIMUHandler)
    # =========================================================================

    @property
    def frame_queue(self) -> Queue:
        return self._frame_q

    @property
    def imu_queue(self) -> Queue:
        return self._imu_q

    @property
    def is_running(self) -> bool:
        return self._running

    def get_stereo_frame(self, timeout: Optional[float] = None) -> Optional[StereoFrame]:
        """Block until a synchronised stereo pair is available."""
        try:
            return self._frame_q.get(timeout=timeout)
        except Empty:
            return None

    def get_imu_sample(self, timeout: Optional[float] = None) -> Optional[IMUSample]:
        """Block until an IMU sample is available."""
        try:
            return self._imu_q.get(timeout=timeout)
        except Empty:
            return None

    def get_latest_frame(self) -> Optional[StereoFrame]:
        with self._lock:
            return self._latest_frame

    def get_latest_imu(self) -> Optional[IMUSample]:
        with self._lock:
            return self._latest_imu

    # =========================================================================
    # Camera intrinsics  (bonus — not part of CameraIMUHandler interface)
    # =========================================================================

    def get_camera_intrinsics(self) -> Optional[Dict[str, Any]]:
        """Return stereo intrinsics parsed from camera_info, or None if not yet received.

        The returned dict contains ready-to-paste ORB-SLAM3 YAML values::

            {
              "left":  { fx, fy, cx, cy, k1, k2, p1, p2, width, height },
              "right": { fx, fy, cx, cy, k1, k2, p1, p2, width, height },
              "baseline_m": <metres>,   # distance between optical centres
              "bf":         <fx * baseline>
            }
        """
        with self._info_lock:
            if self._left_info is None or self._right_info is None:
                return None

            left  = self._left_info
            right = self._right_info

            # The right camera's projection matrix P has P[0,3] = -fx * baseline
            # (negative because the right camera is to the right of the left)
            P_right = right["raw_P"]
            fx_left = left["fx"]
            tx = float(P_right[0, 3])           # = -fx * baseline
            baseline_m = abs(tx / fx_left) if fx_left != 0 else 0.0

            return {
                "left":       {k: left[k]  for k in ("fx","fy","cx","cy","k1","k2","p1","p2","width","height")},
                "right":      {k: right[k] for k in ("fx","fy","cx","cy","k1","k2","p1","p2","width","height")},
                "baseline_m": baseline_m,
                "bf":         fx_left * baseline_m,
            }

    # =========================================================================
    # Lifecycle
    # =========================================================================

    def start(self) -> None:
        """Initialise rclpy, subscribe to all topics, and spin in a daemon thread.

        Raises:
            ImportError: If rclpy is not installed / sourced.
        """
        if self._running:
            logger.warning("IsaacSimCameraIMUHandler already running")
            return

        try:
            import rclpy
            from sensor_msgs.msg import CameraInfo, Image, Imu  # noqa: F401
        except ImportError as exc:
            raise ImportError(
                "rclpy / sensor_msgs not found.\n"
                "Source your ROS 2 installation first:\n"
                "  source /opt/ros/humble/setup.bash\n\n"
                "For cross-machine connectivity to 192.168.2.33 set:\n"
                "  export ROS_DOMAIN_ID=0\n"
                "If multicast is blocked, create ~/cyclone_peers.xml (see module docstring)\n"
                "and export CYCLONEDDS_URI=file://$HOME/cyclone_peers.xml"
            ) from exc

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node(self._node_name)

        self._node.create_subscription(Image,      self._left_topic,       self._on_left_image,  10)
        self._node.create_subscription(Image,      self._right_topic,      self._on_right_image, 10)
        self._node.create_subscription(Imu,        self._imu_topic,        self._on_imu,         50)
        self._node.create_subscription(CameraInfo, self._left_info_topic,  self._on_left_info,    1)
        self._node.create_subscription(CameraInfo, self._right_info_topic, self._on_right_info,   1)

        self._running = True
        self._ros_thread = threading.Thread(
            target=self._spin, daemon=True, name="ros2-spin",
        )
        self._ros_thread.start()

        logger.info(
            "IsaacSimCameraIMUHandler started\n"
            "  images : %s  |  %s\n"
            "  imu    : %s\n"
            "  info   : %s  |  %s",
            self._left_topic, self._right_topic,
            self._imu_topic,
            self._left_info_topic, self._right_info_topic,
        )

    def stop(self) -> None:
        """Shut down the ROS 2 node and spin thread."""
        if not self._running:
            return

        self._running = False

        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None

        if self._ros_thread is not None:
            self._ros_thread.join(timeout=3.0)
            self._ros_thread = None

        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

        logger.info("IsaacSimCameraIMUHandler stopped")

    # =========================================================================
    # ROS 2 callbacks
    # =========================================================================

    def _on_left_image(self, msg: Any) -> None:
        try:
            img = self._preprocess(msg)
            ts  = _ros_stamp_to_monotonic(msg.header)
        except Exception as exc:
            logger.debug("Left image error: %s", exc)
            return
        with self._pending_lock:
            self._pending_left.append((ts, img))
            if len(self._pending_left) > 10:
                self._pending_left.pop(0)
        self._try_sync()

    def _on_right_image(self, msg: Any) -> None:
        try:
            img = self._preprocess(msg)
            ts  = _ros_stamp_to_monotonic(msg.header)
        except Exception as exc:
            logger.debug("Right image error: %s", exc)
            return
        with self._pending_lock:
            self._pending_right.append((ts, img))
            if len(self._pending_right) > 10:
                self._pending_right.pop(0)
        self._try_sync()

    def _on_imu(self, msg: Any) -> None:
        try:
            ts = _ros_stamp_to_monotonic(msg.header)
            sample: IMUSample = {
                "timestamp": ts,
                "accel_x":   msg.linear_acceleration.x / _G_MS2,
                "accel_y":   msg.linear_acceleration.y / _G_MS2,
                "accel_z":   msg.linear_acceleration.z / _G_MS2,
                "gyro_x":    msg.angular_velocity.x * _RAD_TO_DEG,
                "gyro_y":    msg.angular_velocity.y * _RAD_TO_DEG,
                "gyro_z":    msg.angular_velocity.z * _RAD_TO_DEG,
            }
        except Exception as exc:
            logger.debug("IMU error: %s", exc)
            return

        with self._lock:
            self._latest_imu = sample
        self._enqueue(self._imu_q, sample)

    def _on_left_info(self, msg: Any) -> None:
        with self._info_lock:
            self._left_info = _parse_camera_info(msg)
        self._maybe_log_intrinsics()

    def _on_right_info(self, msg: Any) -> None:
        with self._info_lock:
            self._right_info = _parse_camera_info(msg)
        self._maybe_log_intrinsics()

    # =========================================================================
    # Stereo synchronisation
    # =========================================================================

    def _try_sync(self) -> None:
        """Match the closest left/right pair within sync_threshold."""
        with self._pending_lock:
            if not self._pending_left or not self._pending_right:
                return

            best_li = best_ri = -1
            best_dt = float("inf")
            for li, (lt, _) in enumerate(self._pending_left):
                for ri, (rt, _) in enumerate(self._pending_right):
                    dt = abs(lt - rt)
                    if dt < best_dt:
                        best_dt, best_li, best_ri = dt, li, ri

            if best_dt > self._sync_threshold:
                return

            left_ts,  left_img  = self._pending_left.pop(best_li)
            right_ts, right_img = self._pending_right.pop(best_ri)
            ts = (left_ts + right_ts) / 2.0
            frame: StereoFrame = (left_img, right_img, ts)

        with self._lock:
            self._latest_frame = frame
        self._enqueue(self._frame_q, frame)

    # =========================================================================
    # Helpers
    # =========================================================================

    def _preprocess(self, msg: Any) -> np.ndarray:
        img = _ros_image_to_bgr(msg)
        if self._width and self._height:
            h, w = img.shape[:2]
            if (w, h) != (self._width, self._height):
                img = cv2.resize(img, (self._width, self._height),
                                 interpolation=cv2.INTER_LINEAR)
        return img

    def _enqueue(self, q: Queue, item: Any) -> None:
        """Drop-oldest enqueue (never blocks)."""
        try:
            q.put_nowait(item)
        except Full:
            try:
                q.get_nowait()
            except Empty:
                pass
            try:
                q.put_nowait(item)
            except Full:
                pass

    def _spin(self) -> None:
        try:
            import rclpy
            while self._running and rclpy.ok():
                rclpy.spin_once(self._node, timeout_sec=0.1)
        except Exception as exc:
            logger.error("ROS 2 spin error: %s", exc)
        finally:
            self._running = False

    def _maybe_log_intrinsics(self) -> None:
        """Log ORB-SLAM3-ready intrinsics once both camera_info messages arrive."""
        with self._info_lock:
            if self._info_logged or self._left_info is None or self._right_info is None:
                return
            self._info_logged = True
            info = self.get_camera_intrinsics()

        if info is None:
            return

        L, R = info["left"], info["right"]
        logger.info(
            "Camera intrinsics received from sim — paste into stereo_imu_settings.yaml:\n"
            "\n"
            "  # Left camera\n"
            "  Camera1.fx: %.6g\n"
            "  Camera1.fy: %.6g\n"
            "  Camera1.cx: %.6g\n"
            "  Camera1.cy: %.6g\n"
            "  Camera1.k1: %.6g\n"
            "  Camera1.k2: %.6g\n"
            "  Camera1.p1: %.6g\n"
            "  Camera1.p2: %.6g\n"
            "\n"
            "  # Right camera\n"
            "  Camera2.fx: %.6g\n"
            "  Camera2.fy: %.6g\n"
            "  Camera2.cx: %.6g\n"
            "  Camera2.cy: %.6g\n"
            "  Camera2.k1: %.6g\n"
            "  Camera2.k2: %.6g\n"
            "  Camera2.p1: %.6g\n"
            "  Camera2.p2: %.6g\n"
            "\n"
            "  # Stereo\n"
            "  Stereo.b:   %.6g   # baseline metres\n"
            "  Camera.bf:  %.6g   # fx * baseline",
            L["fx"], L["fy"], L["cx"], L["cy"],
            L["k1"], L["k2"], L["p1"], L["p2"],
            R["fx"], R["fy"], R["cx"], R["cy"],
            R["k1"], R["k2"], R["p1"], R["p2"],
            info["baseline_m"], info["bf"],
        )


# ---------------------------------------------------------------------------
# Quick self-test  (python -m src.sensors.isaac_sim_handler)
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    print(
        "Isaac Sim handler self-test\n"
        "Connecting to 192.168.2.33 for 20 s.\n"
        "Prerequisites:\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  export ROS_DOMAIN_ID=0\n"
        "  # if multicast is blocked:\n"
        "  export CYCLONEDDS_URI=file://$HOME/cyclone_peers.xml\n"
    )

    handler = IsaacSimCameraIMUHandler()   # uses confirmed topic defaults

    try:
        handler.start()
    except ImportError as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    deadline = time.monotonic() + 20.0
    frame_count = 0
    imu_count   = 0

    try:
        while time.monotonic() < deadline:
            frame = handler.get_stereo_frame(timeout=0.5)
            if frame is not None:
                left, right, ts = frame
                frame_count += 1
                if frame_count == 1:
                    print(f"  First stereo pair received — shape: {left.shape}")
                if frame_count % 30 == 0:
                    print(f"  [{frame_count:>5} frames]  ts={ts:.3f}")

            imu = handler.get_latest_imu()
            if imu is not None:
                imu_count += 1

    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        handler.stop()
        elapsed = max(0.001, 20.0 - max(0.0, deadline - time.monotonic()))
        print(
            f"\nResult: {frame_count} stereo frames, {imu_count} IMU samples "
            f"in {elapsed:.1f} s  ({frame_count/elapsed:.1f} fps)"
        )
        if frame_count == 0:
            print(
                "\nNo frames received. Checklist:\n"
                "  1. Isaac Sim is running at 192.168.2.33 and Play is pressed\n"
                "  2. ROS 2 bridge extension is enabled in Isaac Sim\n"
                "  3. ros2 topic list shows /left/image_raw\n"
                "  4. ROS_DOMAIN_ID matches on both machines\n"
                "  5. If no multicast: CYCLONEDDS_URI points to the peers XML file"
            )

        # Print intrinsics if received
        info = handler.get_camera_intrinsics()
        if info:
            print(f"\nStereo baseline: {info['baseline_m']*100:.1f} cm")
        else:
            print("\nNo camera_info received yet (check /left/camera_info topic)")
