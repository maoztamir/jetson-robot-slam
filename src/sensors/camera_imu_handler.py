"""Stereo camera and IMU capture handler for IMX219-83 on Jetson Nano.

Provides hardware-accelerated stereo capture via GStreamer ``nvarguscamerasrc``
and concurrent IMU reads over I2C.  Both subsystems run in dedicated daemon
threads that push into bounded, thread-safe queues.

Typical usage::

    handler = CameraIMUHandler()
    handler.start()

    while True:
        frame = handler.get_stereo_frame(timeout=1.0)
        if frame is None:
            continue
        left, right, ts = frame
        imu = handler.get_latest_imu()
        # ... process ...

    handler.stop()
"""

from __future__ import annotations

import logging
import threading
import time
from queue import Empty, Full, Queue
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Default I2C parameters for ICM-20948 on the IMX219-83 camera module
_IMU_I2C_BUS: int = 1
_IMU_ADDRESS: int = 0x68


# ---------------------------------------------------------------------------
# GStreamer pipeline builder
# ---------------------------------------------------------------------------

def build_gstreamer_pipeline(
    sensor_id: int,
    width: int,
    height: int,
    fps: int,
    flip_method: int,
) -> str:
    """Build an nvarguscamerasrc GStreamer pipeline string.

    The pipeline captures at the sensor's native 1280x720, runs through the
    Jetson hardware video converter (``nvvidconv``) to rescale and colour-
    convert, then hands the frame to OpenCV via ``appsink``.

    Args:
        sensor_id: CSI sensor index (0 = left, 1 = right on IMX219-83).
        width:     Desired output width in pixels.
        height:    Desired output height in pixels.
        fps:       Capture frame-rate.
        flip_method: ``nvvidconv`` flip method (0 = none, 2 = rotate-180).

    Returns:
        A GStreamer launch string suitable for ``cv2.VideoCapture``.
    """
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=1280, height=720, "
        f"format=NV12, framerate={fps}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width={width}, height={height}, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! "
        f"appsink drop=1"
    )


# ---------------------------------------------------------------------------
# IMU reader (I2C)
# ---------------------------------------------------------------------------

class _IMUReader:
    """ICM-20948 IMU reader using the Pimoroni ``icm20948`` library.

    If the library is not installed or the physical sensor is unreachable
    the reader falls back to synthetic (mock) data so that the rest of
    the pipeline can still be exercised on a desktop.
    """

    def __init__(
        self,
        bus: int = _IMU_I2C_BUS,
        address: int = _IMU_ADDRESS,
    ) -> None:
        self._bus_id = bus
        self._address = address
        self._imu: Any = None       # ICM20948 instance when available
        self._mock: bool = False

    # -- lifecycle --------------------------------------------------------- #

    def open(self) -> None:
        """Initialise the ICM-20948 sensor."""
        try:
            from icm20948 import ICM20948
            self._imu = ICM20948(i2c_addr=self._address, i2c_bus=self._bus_id)
            self._mock = False
            logger.info(
                "IMU (ICM-20948) opened on I2C bus %d, address 0x%02X",
                self._bus_id, self._address,
            )
        except Exception as exc:
            logger.warning("IMU unavailable (%s) -- using mock data", exc)
            self._imu = None
            self._mock = True

    def close(self) -> None:
        """Release the IMU (no-op for icm20948 library)."""
        self._imu = None

    # -- reading ----------------------------------------------------------- #

    def read(self) -> Dict[str, float]:
        """Return a single IMU sample as a dict.

        Returns:
            Dict with keys ``timestamp``, ``accel_x``, ``accel_y``,
            ``accel_z``, ``gyro_x``, ``gyro_y``, ``gyro_z``.
            Accelerometer values are in *g*, gyroscope in *deg/s*.
            If the physical sensor is unavailable the values are synthetic.
        """
        if self._mock:
            return self._mock_reading()
        try:
            return self._hw_reading()
        except Exception as exc:
            logger.debug("IMU read failed (%s) -- returning mock", exc)
            return self._mock_reading()

    def _hw_reading(self) -> Dict[str, float]:
        """Read accelerometer and gyroscope via icm20948 library."""
        ax, ay, az, gx, gy, gz = self._imu.read_accelerometer_gyro_data()

        return {
            "timestamp": time.monotonic(),
            "accel_x": ax,
            "accel_y": ay,
            "accel_z": az,
            "gyro_x": gx,
            "gyro_y": gy,
            "gyro_z": gz,
        }

    @staticmethod
    def _mock_reading() -> Dict[str, float]:
        """Return a plausible resting IMU sample."""
        return {
            "timestamp": time.monotonic(),
            "accel_x": 0.0,
            "accel_y": 0.0,
            "accel_z": 1.0,   # 1 g pointing down at rest
            "gyro_x": 0.0,
            "gyro_y": 0.0,
            "gyro_z": 0.0,
        }


# ---------------------------------------------------------------------------
# Type aliases
# ---------------------------------------------------------------------------

StereoFrame = Tuple[np.ndarray, np.ndarray, float]
"""``(left_bgr, right_bgr, monotonic_timestamp)``"""

IMUSample = Dict[str, float]
"""Dict with keys: timestamp, accel_x/y/z, gyro_x/y/z."""


# ---------------------------------------------------------------------------
# Public handler
# ---------------------------------------------------------------------------

class CameraIMUHandler:
    """Concurrent stereo-camera and IMU capture for Jetson Nano.

    Data is exposed through two bounded queues **plus** ``get_latest_*``
    convenience methods.  When a queue is full the *oldest* item is
    silently discarded so that consumers always see the freshest data.

    Args:
        width:            Output frame width in pixels.
        height:           Output frame height in pixels.
        fps:              Capture frame-rate.
        flip_method:      ``nvvidconv`` flip method passed to GStreamer.
        frame_queue_size: Max buffered stereo pairs (small keeps latency low).
        imu_queue_size:   Max buffered IMU samples.
        enable_imu:       Set ``False`` to skip IMU initialisation entirely.
    """

    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        flip_method: int = 0,
        frame_queue_size: int = 2,
        imu_queue_size: int = 10,
        enable_imu: bool = True,
    ) -> None:
        self._width = width
        self._height = height
        self._fps = fps
        self._flip = flip_method
        self._enable_imu = enable_imu

        # Queues
        self._frame_q: Queue[StereoFrame] = Queue(maxsize=frame_queue_size)
        self._imu_q: Queue[IMUSample] = Queue(maxsize=imu_queue_size)

        # Latest-value cache (lock-protected)
        self._lock = threading.Lock()
        self._latest_frame: Optional[StereoFrame] = None
        self._latest_imu: Optional[IMUSample] = None

        # Internal state
        self._left_cap: Optional[cv2.VideoCapture] = None
        self._right_cap: Optional[cv2.VideoCapture] = None
        self._imu_reader: Optional[_IMUReader] = None

        self._running = False
        self._cam_thread: Optional[threading.Thread] = None
        self._imu_thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------ public

    @property
    def frame_queue(self) -> Queue[StereoFrame]:
        """Bounded queue of ``(left, right, timestamp)`` tuples."""
        return self._frame_q

    @property
    def imu_queue(self) -> Queue[IMUSample]:
        """Bounded queue of IMU sample dicts."""
        return self._imu_q

    @property
    def is_running(self) -> bool:
        """``True`` while capture threads are active."""
        return self._running

    # -- blocking access --------------------------------------------------- #

    def get_stereo_frame(
        self,
        timeout: Optional[float] = None,
    ) -> Optional[StereoFrame]:
        """Block until a stereo pair is available or *timeout* expires.

        Args:
            timeout: Max seconds to wait.  ``None`` blocks indefinitely.

        Returns:
            ``(left, right, timestamp)`` or ``None`` on timeout.
        """
        try:
            return self._frame_q.get(timeout=timeout)
        except Empty:
            return None

    def get_imu_sample(
        self,
        timeout: Optional[float] = None,
    ) -> Optional[IMUSample]:
        """Block until an IMU sample is available or *timeout* expires.

        Args:
            timeout: Max seconds to wait.  ``None`` blocks indefinitely.

        Returns:
            IMU dict or ``None`` on timeout.
        """
        try:
            return self._imu_q.get(timeout=timeout)
        except Empty:
            return None

    # -- non-blocking access ----------------------------------------------- #

    def get_latest_frame(self) -> Optional[StereoFrame]:
        """Return the most recent stereo pair without blocking.

        Returns:
            ``(left, right, timestamp)`` or ``None`` if no frame has
            been captured yet.
        """
        with self._lock:
            return self._latest_frame

    def get_latest_imu(self) -> Optional[IMUSample]:
        """Return the most recent IMU reading without blocking.

        Returns:
            IMU dict or ``None`` if no sample has been read yet.
        """
        with self._lock:
            return self._latest_imu

    # -- lifecycle --------------------------------------------------------- #

    def start(self) -> None:
        """Open cameras (and optionally the IMU) and launch capture threads.

        Raises:
            RuntimeError: If either camera fails to open via GStreamer.
        """
        if self._running:
            logger.warning("CameraIMUHandler already running")
            return

        self._open_cameras()

        if self._enable_imu:
            self._imu_reader = _IMUReader()
            self._imu_reader.open()

        self._running = True

        self._cam_thread = threading.Thread(
            target=self._camera_loop, daemon=True, name="stereo-capture",
        )
        self._cam_thread.start()

        if self._enable_imu:
            self._imu_thread = threading.Thread(
                target=self._imu_loop, daemon=True, name="imu-capture",
            )
            self._imu_thread.start()

        logger.info(
            "Capture started -- %dx%d @ %d fps, IMU %s",
            self._width, self._height, self._fps,
            "enabled" if self._enable_imu else "disabled",
        )

    def stop(self) -> None:
        """Signal threads to exit and release all hardware resources."""
        if not self._running:
            return

        self._running = False

        if self._cam_thread is not None:
            self._cam_thread.join(timeout=3.0)
            self._cam_thread = None
        if self._imu_thread is not None:
            self._imu_thread.join(timeout=2.0)
            self._imu_thread = None

        if self._left_cap is not None:
            self._left_cap.release()
            self._left_cap = None
        if self._right_cap is not None:
            self._right_cap.release()
            self._right_cap = None

        if self._imu_reader is not None:
            self._imu_reader.close()
            self._imu_reader = None

        logger.info("Capture stopped -- all resources released")

    # --------------------------------------------------------------- private

    def _open_cameras(self) -> None:
        """Create GStreamer-backed VideoCapture objects for both sensors."""
        left_pipe = build_gstreamer_pipeline(
            0, self._width, self._height, self._fps, self._flip,
        )
        right_pipe = build_gstreamer_pipeline(
            1, self._width, self._height, self._fps, self._flip,
        )

        self._left_cap = cv2.VideoCapture(left_pipe, cv2.CAP_GSTREAMER)
        self._right_cap = cv2.VideoCapture(right_pipe, cv2.CAP_GSTREAMER)

        if not self._left_cap.isOpened():
            raise RuntimeError(
                "Failed to open LEFT camera (sensor-id=0). "
                "Verify CSI connection: "
                "gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! fakesink"
            )
        if not self._right_cap.isOpened():
            self._left_cap.release()
            raise RuntimeError(
                "Failed to open RIGHT camera (sensor-id=1). "
                "Verify CSI connection: "
                "gst-launch-1.0 nvarguscamerasrc sensor-id=1 ! fakesink"
            )

    # -- capture loops ----------------------------------------------------- #

    def _enqueue(self, q: Queue, item: Any) -> None:
        """Non-blocking put that drops the oldest entry when full."""
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

    def _camera_loop(self) -> None:
        """Grab synchronised stereo pairs and push to the frame queue."""
        while self._running:
            ok_l, left = self._left_cap.read()
            ok_r, right = self._right_cap.read()

            if not ok_l or not ok_r:
                logger.warning("Stereo grab failed -- retrying")
                time.sleep(0.005)
                continue

            ts = time.monotonic()
            frame: StereoFrame = (left, right, ts)

            with self._lock:
                self._latest_frame = frame

            self._enqueue(self._frame_q, frame)

    def _imu_loop(self) -> None:
        """Poll the IMU at ~200 Hz and push to the IMU queue."""
        period = 1.0 / 200.0
        while self._running:
            sample = self._imu_reader.read()

            with self._lock:
                self._latest_imu = sample

            self._enqueue(self._imu_q, sample)

            time.sleep(period)


# ---------------------------------------------------------------------------
# Quick self-test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    handler = CameraIMUHandler()
    handler.start()

    deadline = time.monotonic() + 10.0
    frame_count = 0
    imu_count = 0

    print("Running for 10 seconds -- press Ctrl+C to stop early\n")

    try:
        while time.monotonic() < deadline:
            frame = handler.get_stereo_frame(timeout=0.5)
            if frame is not None:
                left, right, ts = frame
                frame_count += 1
                if frame_count % 30 == 0:
                    print(
                        f"[frame {frame_count:>5}] "
                        f"left={left.shape}  right={right.shape}  "
                        f"ts={ts:.3f}"
                    )

            imu = handler.get_latest_imu()
            if imu is not None:
                imu_count += 1
                if imu_count % 200 == 0:
                    print(
                        f"[imu   {imu_count:>5}] "
                        f"accel=({imu['accel_x']:+.3f}, "
                        f"{imu['accel_y']:+.3f}, "
                        f"{imu['accel_z']:+.3f})  "
                        f"gyro=({imu['gyro_x']:+.3f}, "
                        f"{imu['gyro_y']:+.3f}, "
                        f"{imu['gyro_z']:+.3f})"
                    )

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    finally:
        handler.stop()
        elapsed = 10.0 - max(0.0, deadline - time.monotonic())
        print(
            f"\nDone -- {frame_count} frames, {imu_count} IMU reads "
            f"in {elapsed:.1f}s"
        )
        if elapsed > 0:
            print(
                f"       {frame_count / elapsed:.1f} fps, "
                f"{imu_count / elapsed:.1f} IMU samples/s"
            )
