"""ORB_SLAM3 stereo-inertial wrapper for Jetson Nano.

Provides a clean Python interface around the ORB_SLAM3 C++ library.
When the ``orbslam3`` Python bindings are not installed a **mock backend**
generates a synthetic figure-eight trajectory so the rest of the pipeline
can be developed and tested on any machine.

State machine::

    INITIALIZING ──> TRACKING ──> LOST
         ^               |          |
         |               v          |
         +───────────────+──────────+

Memory budget (4 GB Jetson):
    * Trajectory is stored in a bounded ``collections.deque`` (default 1000
      poses).  Older entries are silently discarded.
    * Map points are fetched on demand — never cached in Python.
    * ``frame_skip`` lets callers trade accuracy for throughput.

Typical usage::

    slam = ORBSLAM3Wrapper("ORBvoc.txt", "settings.yaml")
    slam.initialise()

    pose = slam.process_frame(left, right, ts, imu_samples)
    if pose is not None:
        publish(pose)

    slam.save_trajectory("trajectory.txt")
    slam.shutdown()
"""

from __future__ import annotations

import contextlib
import enum
import logging
import math
import os
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, Generator, List, Optional, Sequence, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# Type alias — matches ``src.sensors.camera_imu_handler.IMUSample``.
IMUSample = Dict[str, float]


@contextlib.contextmanager
def _suppress_cpp_stdout() -> Generator[None, None, None]:
    """Redirect fd 1 (stdout) to /dev/null for the duration of the block.

    ORB_SLAM3's C++ code writes diagnostic noise directly to std::cout
    (e.g. "LM: Active map reset", "TRACK_REF_KF: Less than 15 matches").
    These bypass Python's logging system and pollute the console.
    Redirecting the underlying file descriptor silences them without
    affecting Python's ``sys.stdout``.
    """
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    saved_fd = os.dup(1)
    try:
        os.dup2(devnull_fd, 1)
        yield
    finally:
        os.dup2(saved_fd, 1)
        os.close(saved_fd)
        os.close(devnull_fd)


# ---------------------------------------------------------------------------
# Tracking state
# ---------------------------------------------------------------------------

class TrackingState(enum.IntEnum):
    """Tracking states exposed by the wrapper.

    Values 0-2 mirror the ORB_SLAM3 C++ enum so that callers can compare
    directly against the raw integer when the real library is loaded.
    """

    INITIALIZING = 0
    TRACKING = 2
    LOST = 3


# Internal ORB_SLAM3 ``Tracking::eTrackingState`` value that means "OK".
_ORB_OK: int = 2


# ---------------------------------------------------------------------------
# Rotation matrix -> quaternion
# ---------------------------------------------------------------------------

def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a unit quaternion ``(w, x, y, z)``.

    Uses the Shepperd method which is numerically stable for all
    rotation magnitudes.

    Args:
        R: 3x3 orthogonal rotation matrix.

    Returns:
        Length-4 NumPy array ``[w, x, y, z]``, unit-normalised.
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

    q = np.array([w, x, y, z], dtype=np.float64)
    q /= np.linalg.norm(q)
    return q


# ---------------------------------------------------------------------------
# Wrapper
# ---------------------------------------------------------------------------

class ORBSLAM3Wrapper:
    """Thread-safe ORB_SLAM3 stereo-inertial wrapper.

    Args:
        vocab_path:          Path to the ORB vocabulary
                             (``ORBvoc.txt`` or ``.bin``).
        settings_path:       Path to the ORB_SLAM3 YAML settings file.
        use_imu:             ``True`` for STEREO_INERTIAL mode, ``False``
                             for plain STEREO.
        trajectory_maxlen:   Maximum poses kept in memory.  Older entries
                             are evicted FIFO to bound RAM on the Jetson.
        frame_skip:          Process every *N*-th frame (1 = no skip).
                             Skipped frames return the most recent pose.
    """

    def __init__(
        self,
        vocab_path: str,
        settings_path: str,
        *,
        use_imu: bool = True,
        trajectory_maxlen: int = 1000,
        frame_skip: int = 1,
    ) -> None:
        self._vocab_path = vocab_path
        self._settings_path = settings_path
        self._use_imu = use_imu
        self._frame_skip = max(1, frame_skip)

        self._slam: Any = None
        self._mock: bool = False
        self._state = TrackingState.INITIALIZING

        self._lock = threading.Lock()
        self._trajectory: Deque[Tuple[float, np.ndarray]] = deque(
            maxlen=trajectory_maxlen,
        )
        self._frame_count: int = 0
        self._track_count: int = 0
        self._started: bool = False

    # ---------------------------------------------------------------- public

    @property
    def state(self) -> TrackingState:
        """Current tracking state."""
        return self._state

    @property
    def frame_count(self) -> int:
        """Total frames submitted (including skipped)."""
        return self._frame_count

    @property
    def track_count(self) -> int:
        """Frames actually processed by ORB_SLAM3."""
        return self._track_count

    @property
    def is_started(self) -> bool:
        """``True`` after :meth:`initialise` succeeds."""
        return self._started

    # -- lifecycle --------------------------------------------------------- #

    def initialise(self) -> None:
        """Create the ORB_SLAM3 ``System`` object.

        Falls back to the mock backend when the ``orbslam3`` bindings
        cannot be imported.

        Raises:
            FileNotFoundError: If the vocabulary or settings file is
                missing **and** the real bindings are installed (the
                mock backend ignores paths).
        """
        if self._started:
            logger.warning("ORBSLAM3Wrapper already initialised")
            return

        try:
            self._init_real()
            self._mock = False
        except ImportError:
            logger.warning(
                "orbslam3 Python bindings not found -- "
                "running MOCK backend (synthetic trajectory)"
            )
            self._slam = None
            self._mock = True

        self._state = TrackingState.INITIALIZING
        self._started = True

        mode = "stereo-inertial" if self._use_imu else "stereo"
        backend = "ORB_SLAM3" if not self._mock else "MOCK"
        logger.info("SLAM initialised -- %s / %s", backend, mode)

    def shutdown(self) -> None:
        """Shut down ORB_SLAM3 and release resources."""
        if not self._started:
            return

        if self._slam is not None:
            try:
                self._slam.shutdown()
                logger.info("ORB_SLAM3 shut down cleanly")
            except Exception:
                logger.debug("ORB_SLAM3 shutdown error", exc_info=True)
            self._slam = None

        self._started = False
        self._state = TrackingState.INITIALIZING
        logger.info(
            "Wrapper shut down -- %d/%d frames tracked, %d poses stored",
            self._track_count, self._frame_count, len(self._trajectory),
        )

    # -- frame processing -------------------------------------------------- #

    def process_frame(
        self,
        left: np.ndarray,
        right: np.ndarray,
        timestamp: float,
        imu_samples: Optional[Sequence[IMUSample]] = None,
    ) -> Optional[np.ndarray]:
        """Submit a stereo pair and return the 4x4 world-frame pose.

        Args:
            left:        Left BGR image.
            right:       Right BGR image (same resolution).
            timestamp:   Monotonic time of capture in seconds.
            imu_samples: IMU dicts accumulated since the *previous*
                         call.  Each dict must contain at minimum
                         ``timestamp``, ``accel_x/y/z``, ``gyro_x/y/z``.
                         Ignored when ``use_imu=False``.

        Returns:
            4x4 ``numpy.ndarray`` homogeneous transform ``T_wc``
            (world <- camera) when tracking succeeds, or ``None`` when
            tracking is lost or the system is still initialising.
        """
        if not self._started:
            return None

        self._frame_count += 1

        # Frame skipping: return cached pose instead of processing.
        if self._frame_count % self._frame_skip != 0:
            return self._get_latest_pose_matrix()

        if self._mock:
            Twc = self._mock_process(timestamp)
        else:
            Twc = self._real_process(left, right, timestamp, imu_samples)

        if Twc is None:
            self._state = TrackingState.LOST
            return None

        self._state = TrackingState.TRACKING
        self._track_count += 1

        with self._lock:
            self._trajectory.append((timestamp, Twc.copy()))

        return Twc

    # -- queries ----------------------------------------------------------- #

    def get_latest_pose(self) -> Optional[Tuple[float, np.ndarray]]:
        """Return ``(timestamp, T_wc)`` for the most recent tracked pose.

        Returns:
            Tuple of (timestamp, 4x4 array) or ``None`` if nothing has
            been tracked yet.
        """
        with self._lock:
            if self._trajectory:
                ts, T = self._trajectory[-1]
                return (ts, T.copy())
        return None

    def get_trajectory(self) -> List[Tuple[float, np.ndarray]]:
        """Return the cached trajectory as ``[(timestamp, T_wc), ...]``.

        The list length is bounded by *trajectory_maxlen*.  Each matrix
        is a copy so callers may modify them freely.

        Returns:
            List of ``(monotonic_timestamp, 4x4_pose)`` tuples.
        """
        with self._lock:
            return [(ts, T.copy()) for ts, T in self._trajectory]

    def get_map_points(self) -> np.ndarray:
        """Return the current 3-D map points as an ``(N, 3)`` array.

        On the mock backend this returns a small synthetic point cloud
        (200 points in a 4x4x1 m box, seeded for reproducibility).

        Returns:
            ``float32`` array of shape ``(N, 3)``.  May be empty.
        """
        if self._mock:
            return self._mock_map_points()

        if self._slam is None:
            return np.empty((0, 3), dtype=np.float32)

        try:
            raw = self._slam.get_tracked_mappoints()
            if raw is None or len(raw) == 0:
                return np.empty((0, 3), dtype=np.float32)
            return np.asarray(raw, dtype=np.float32)
        except Exception:
            logger.debug("get_map_points failed", exc_info=True)
            return np.empty((0, 3), dtype=np.float32)

    def save_trajectory(self, path: str) -> int:
        """Write the trajectory to a TUM-format text file.

        TUM format per line::

            timestamp tx ty tz qx qy qz qw

        Args:
            path: Destination file path (parent dirs created if needed).

        Returns:
            Number of poses written.
        """
        traj = self.get_trajectory()
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)

        with open(out, "w") as fh:
            fh.write("# TUM trajectory format\n")
            fh.write("# timestamp tx ty tz qx qy qz qw\n")
            for ts, T in traj:
                t = T[:3, 3]
                q = rotation_matrix_to_quaternion(T[:3, :3])
                fh.write(
                    f"{ts:.6f} "
                    f"{t[0]:.6f} {t[1]:.6f} {t[2]:.6f} "
                    f"{q[1]:.6f} {q[2]:.6f} {q[3]:.6f} {q[0]:.6f}\n"
                )

        logger.info("Saved %d poses to %s", len(traj), out)
        return len(traj)

    # --------------------------------------------------------------- private

    def _get_latest_pose_matrix(self) -> Optional[np.ndarray]:
        """Return the last cached pose matrix, or ``None``."""
        with self._lock:
            if self._trajectory:
                return self._trajectory[-1][1].copy()
        return None

    def _init_real(self) -> None:
        """Initialise the real ORB_SLAM3 system.

        Raises ``ImportError`` when the bindings are absent.
        """
        import orbslam3  # noqa: F811

        vocab = Path(self._vocab_path)
        settings = Path(self._settings_path)
        if not vocab.is_file():
            raise FileNotFoundError(f"ORB vocabulary not found: {vocab}")
        if not settings.is_file():
            raise FileNotFoundError(f"SLAM settings not found: {settings}")

        sensor = (
            orbslam3.Sensor.STEREO_INERTIAL if self._use_imu
            else orbslam3.Sensor.STEREO
        )
        with _suppress_cpp_stdout():
            self._slam = orbslam3.System(
                str(vocab), str(settings), sensor, use_viewer=False,
            )

    # -- real backend ------------------------------------------------------ #

    def _real_process(
        self,
        left: np.ndarray,
        right: np.ndarray,
        timestamp: float,
        imu_samples: Optional[Sequence[IMUSample]],
    ) -> Optional[np.ndarray]:
        """Run a frame through the real ORB_SLAM3 pipeline."""
        try:
            with _suppress_cpp_stdout():
                if self._use_imu and imu_samples:
                    imu_arr = self._imu_samples_to_array(imu_samples)
                    Tcw = self._slam.process_image_stereo_inertial(
                        left, right, timestamp, imu_arr,
                    )
                else:
                    Tcw = self._slam.process_image_stereo(
                        left, right, timestamp,
                    )

            state = self._slam.get_tracking_state()
            if Tcw is None or state != _ORB_OK:
                return None

            # ORB_SLAM3 returns T_cw (camera <- world).  Invert to get
            # the camera position in the world frame: T_wc = T_cw^{-1}.
            Twc = np.eye(4, dtype=np.float64)
            R = Tcw[:3, :3]
            t = Tcw[:3, 3]
            Twc[:3, :3] = R.T
            Twc[:3, 3] = -R.T @ t
            return Twc

        except Exception:
            logger.error("ORB_SLAM3 processing error", exc_info=True)
            return None

    @staticmethod
    def _imu_samples_to_array(samples: Sequence[IMUSample]) -> np.ndarray:
        """Convert IMU sample dicts to the Nx7 array ORB_SLAM3 expects.

        Columns: ``[timestamp, ax, ay, az, gx, gy, gz]``.
        """
        rows = [
            [
                s["timestamp"],
                s["accel_x"], s["accel_y"], s["accel_z"],
                s["gyro_x"], s["gyro_y"], s["gyro_z"],
            ]
            for s in samples
        ]
        return np.array(rows, dtype=np.float64)

    # -- mock backend ------------------------------------------------------ #

    def _mock_process(self, timestamp: float) -> Optional[np.ndarray]:
        """Generate a deterministic figure-eight trajectory for testing.

        The first few frames return identity (simulating INITIALIZING),
        then the pose traces a smooth Lissajous curve in the XY plane.
        """
        n = self._track_count + 1

        # Simulate initialisation delay.
        if n < 5:
            self._state = TrackingState.INITIALIZING
            return np.eye(4, dtype=np.float64)

        t = n * 0.05
        x = 2.0 * math.sin(t * 0.5)
        y = 1.5 * math.sin(t * 1.0)
        z = 0.0

        # Heading tangent to the path.
        dx = 2.0 * 0.5 * math.cos(t * 0.5)
        dy = 1.5 * 1.0 * math.cos(t * 1.0)
        heading = math.atan2(dy, dx)
        c, s = math.cos(heading), math.sin(heading)

        T = np.eye(4, dtype=np.float64)
        T[0, 0] = c;   T[0, 1] = -s
        T[1, 0] = s;   T[1, 1] = c
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        return T

    @staticmethod
    def _mock_map_points() -> np.ndarray:
        """Generate a small reproducible point cloud for testing."""
        rng = np.random.RandomState(42)
        return rng.uniform(
            [-2, -2, -0.5], [2, 2, 0.5], size=(200, 3),
        ).astype(np.float32)


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    slam = ORBSLAM3Wrapper(
        vocab_path="not_needed_for_mock",
        settings_path="not_needed_for_mock",
        use_imu=True,
        trajectory_maxlen=1000,
        frame_skip=1,
    )
    slam.initialise()

    # Synthetic stereo pair.
    dummy = np.zeros((480, 640, 3), dtype=np.uint8)
    n_frames = 300

    print(f"Processing {n_frames} mock frames...\n")
    t0 = time.monotonic()

    for i in range(n_frames):
        ts = time.monotonic()

        # Synthesise one IMU measurement per frame.
        imu = {
            "timestamp": ts,
            "accel_x": 0.0, "accel_y": 0.0, "accel_z": 1.0,
            "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.05,
        }

        pose = slam.process_frame(dummy, dummy, ts, [imu])

        if pose is not None and (i + 1) % 30 == 0:
            t = pose[:3, 3]
            q = rotation_matrix_to_quaternion(pose[:3, :3])
            print(
                f"  frame {i + 1:>4}  "
                f"state={slam.state.name:<14s}  "
                f"pos=({t[0]:+6.3f}, {t[1]:+6.3f}, {t[2]:+6.3f})  "
                f"quat=({q[0]:+.4f}, {q[1]:+.4f}, "
                f"{q[2]:+.4f}, {q[3]:+.4f})"
            )

    elapsed = time.monotonic() - t0

    # Trajectory and map.
    traj = slam.get_trajectory()
    pts = slam.get_map_points()
    saved = slam.save_trajectory("/tmp/slam_mock_trajectory.txt")

    slam.shutdown()

    print(f"\n--- Results ---")
    print(f"Frames submitted : {slam.frame_count}")
    print(f"Frames tracked   : {slam.track_count}")
    print(f"Trajectory poses : {len(traj)}")
    print(f"Map points       : {pts.shape[0]}")
    print(f"Saved to file    : {saved} poses")
    print(f"Throughput       : {n_frames / elapsed:.0f} fps "
          f"({elapsed:.3f}s total)")

    # Quick sanity check on the last pose.
    if traj:
        last_ts, last_T = traj[-1]
        print(f"\nLast pose (T_wc):\n{last_T}")
