"""Main entry point for the Jetson robot SLAM pipeline.

Integrates stereo camera capture, ORB_SLAM3 processing, and AWS IoT
publishing into a single run-loop with graceful shutdown, periodic
performance logging, and rotating file handlers.

Usage::

    python -m src.main
    python -m src.main --config config/my_config.yaml --verbose
    python -m src.main --no-aws --visualize
"""

from __future__ import annotations

# Preload libgomp before any other import to avoid aarch64 TLS allocation error.
import ctypes, ctypes.util  # noqa: E401
for _lib in ("/usr/lib/aarch64-linux-gnu/libgomp.so.1", "gomp"):
    try:
        ctypes.CDLL(_lib, mode=ctypes.RTLD_GLOBAL)
        break
    except OSError:
        pass

import argparse
import datetime
import json
import logging
import math
import os
import signal
import sys
import time
from logging.handlers import RotatingFileHandler
from pathlib import Path
from queue import Empty
from typing import Any, Dict, List, Optional

import numpy as np
import yaml

from src.sensors.camera_imu_handler import CameraIMUHandler, IMUSample
from src.sensors.isaac_sim_handler import IsaacSimCameraIMUHandler
from src.slam.orb_slam3_wrapper import ORBSLAM3Wrapper, TrackingState
from src.slam.dead_reckoning import DeadReckoningIntegrator
from src.cloud.aws_iot_publisher import AWSIoTPublisher

logger = logging.getLogger("robot")

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
_DEFAULT_CONFIG = _PROJECT_ROOT / "config" / "default_config.yaml"

# Maximum SLAM init retries before falling back to mock.
_SLAM_INIT_RETRIES: int = 3
_SLAM_INIT_DELAY: float = 2.0  # seconds between retries


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

def _load_config(path: str) -> Dict[str, Any]:
    """Load a YAML config file, falling back to ``default_config.yaml``.

    Args:
        path: Caller-supplied path (may not exist).

    Returns:
        Parsed dict.  Empty dict if nothing is found.
    """
    candidates = [Path(path), _DEFAULT_CONFIG]
    for p in candidates:
        if p.is_file():
            with open(p) as fh:
                cfg = yaml.safe_load(fh) or {}
            logger.info("Loaded config from %s", p)
            return cfg
    logger.warning("No config file found -- using built-in defaults")
    return {}


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

def _setup_logging(cfg: Dict[str, Any], verbose: bool) -> None:
    """Configure root logger with console + rotating file handlers.

    Args:
        cfg: ``logging`` section from the YAML config.
        verbose: Override log level to DEBUG.
    """
    log_cfg = cfg.get("logging", {})
    level_name = "DEBUG" if verbose else log_cfg.get("level", "INFO")
    level = getattr(logging, level_name, logging.INFO)

    fmt = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    datefmt = "%Y-%m-%d %H:%M:%S"
    formatter = logging.Formatter(fmt, datefmt)

    root = logging.getLogger()
    root.setLevel(level)
    # Avoid duplicate handlers on repeated calls.
    root.handlers.clear()

    # Console handler.
    console = logging.StreamHandler(sys.stdout)
    console.setLevel(level)
    console.setFormatter(formatter)
    root.addHandler(console)

    # Rotating file handler.
    log_file = log_cfg.get("file", "logs/robot.log")
    log_path = _PROJECT_ROOT / log_file
    log_path.parent.mkdir(parents=True, exist_ok=True)

    file_h = RotatingFileHandler(
        str(log_path),
        maxBytes=log_cfg.get("max_bytes", 10_485_760),
        backupCount=log_cfg.get("backup_count", 5),
    )
    file_h.setLevel(level)
    file_h.setFormatter(formatter)
    root.addHandler(file_h)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Jetson robot -- stereo-inertial SLAM with AWS IoT",
    )
    _LOCAL_CONFIG = _PROJECT_ROOT / "config" / "local_config.yaml"
    p.add_argument(
        "--config",
        default=str(_LOCAL_CONFIG if _LOCAL_CONFIG.is_file() else _DEFAULT_CONFIG),
        help="Path to YAML config (default: config/local_config.yaml if present, else default_config.yaml)",
    )
    p.add_argument(
        "--verbose", action="store_true",
        help="Set log level to DEBUG",
    )
    p.add_argument(
        "--no-aws", action="store_true",
        help="Disable AWS IoT publishing (SLAM only)",
    )
    p.add_argument(
        "--visualize", action="store_true",
        help="Show live stereo camera feed in an OpenCV window",
    )
    p.add_argument(
        "--telemetry-file",
        default="",
        help=(
            "Path to local JSONL file for saving telemetry. "
            "Defaults to telemetry/{device_id}_{YYYY-MM-DDTHH-MM-SS}.jsonl "
            "(one file per run, named by device and start time)."
        ),
    )
    p.add_argument(
        "--mock", action="store_true",
        help="Run with synthetic camera and IMU data (no hardware required)",
    )
    p.add_argument(
        "--mode",
        choices=["slam_imu", "imu_only"],
        default="slam_imu",
        help=(
            "slam_imu (default): run SLAM and fall back to IMU-only when tracking fails; "
            "imu_only: skip SLAM entirely and publish only IMU data"
        ),
    )
    p.add_argument(
        "--debug-imu", action="store_true",
        help="Print live acceleration, gyro, and tilt data to the terminal (one line, overwrites in place)",
    )
    p.add_argument(
        "--sim", action="store_true",
        help="Use Isaac Sim via ROS 2 instead of physical CSI camera / I2C IMU",
    )
    p.add_argument(
        "--sim-left-topic", default="/left/image_raw",
        help="ROS 2 topic for left camera (sim mode only)",
    )
    p.add_argument(
        "--sim-right-topic", default="/right/image_raw",
        help="ROS 2 topic for right camera (sim mode only)",
    )
    p.add_argument(
        "--sim-imu-topic", default="/imu",
        help="ROS 2 topic for IMU (sim mode only)",
    )
    return p.parse_args()


# ---------------------------------------------------------------------------
# Performance monitor
# ---------------------------------------------------------------------------

class _PerfMonitor:
    """Lightweight FPS / queue / memory tracker.

    Accumulates per-interval counts and emits a single INFO log line
    every *interval* seconds.
    """

    def __init__(self, interval: float = 30.0) -> None:
        self._interval = interval
        self._frames: int = 0
        self._slam_ok: int = 0
        self._slam_fail: int = 0
        self._last_report = time.monotonic()

    def tick_frame(self) -> None:
        """Call once per camera frame received."""
        self._frames += 1

    def tick_slam(self, ok: bool) -> None:
        """Call once per SLAM ``process_frame`` call."""
        if ok:
            self._slam_ok += 1
        else:
            self._slam_fail += 1

    def maybe_report(
        self,
        cam_qsize: int,
        imu_qsize: int,
        slam: Optional[ORBSLAM3Wrapper],
        publisher: Optional[AWSIoTPublisher],
    ) -> None:
        """Emit a log line if *interval* seconds have elapsed."""
        now = time.monotonic()
        dt = now - self._last_report
        if dt < self._interval:
            return

        fps = self._frames / dt if dt > 0 else 0.0
        slam_total = self._slam_ok + self._slam_fail
        slam_pct = (
            (self._slam_ok / slam_total * 100.0) if slam_total > 0 else 0.0
        )
        mem_mb = _get_rss_mb()

        parts = [
            f"fps={fps:.1f}",
        ]
        if slam is not None:
            parts.append(f"slam={slam_pct:.0f}%({self._slam_ok}/{slam_total})")
            parts.append(f"state={slam.state.name}")
        else:
            parts.append("slam=disabled")
        parts += [
            f"cam_q={cam_qsize}",
            f"imu_q={imu_qsize}",
            f"mem={mem_mb:.0f}MB",
        ]
        if publisher is not None:
            parts.append(f"pub={publisher.publish_count}")
            parts.append(f"err={publisher.error_count}")

        logger.info("PERF  %s", "  ".join(parts))

        # Reset interval counters.
        self._frames = 0
        self._slam_ok = 0
        self._slam_fail = 0
        self._last_report = now


def _get_rss_mb() -> float:
    """Read the current process RSS from ``/proc/self/status``."""
    try:
        with open("/proc/self/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) / 1024.0
    except Exception:
        pass
    return 0.0


# ---------------------------------------------------------------------------
# Component construction
# ---------------------------------------------------------------------------

def _build_camera(cfg: Dict[str, Any], mock: bool = False, enable_stereo: bool = True) -> CameraIMUHandler:
    cam = cfg.get("camera", {})
    res = cam.get("resolution", [640, 480])
    return CameraIMUHandler(
        width=res[0],
        height=res[1],
        fps=cam.get("fps", 30),
        flip_method=cam.get("flip_method", 0),
        enable_imu=True,
        enable_stereo=enable_stereo,
        mock=mock,
        backend=cam.get("backend", "jetson"),
        rpi5_cam1_name=cam.get(
            "rpi5_cam1_name",
            "/base/axi/pcie@1000120000/rp1/i2c@80000/imx219@10",
        ),
    )


def _build_sim_camera(
    cfg: Dict[str, Any],
    left_topic: str,
    right_topic: str,
    imu_topic: str,
) -> IsaacSimCameraIMUHandler:
    """Build an Isaac Sim handler using ROS 2 topic names from CLI args."""
    cam = cfg.get("camera", {})
    res = cam.get("resolution", [640, 480])
    sim_cfg = cfg.get("isaac_sim", {})
    return IsaacSimCameraIMUHandler(
        left_topic=sim_cfg.get("left_topic", left_topic),
        right_topic=sim_cfg.get("right_topic", right_topic),
        imu_topic=sim_cfg.get("imu_topic", imu_topic),
        width=res[0],
        height=res[1],
        sync_threshold=sim_cfg.get("sync_threshold_ms", 20) / 1000.0,
    )


def _build_slam(cfg: Dict[str, Any]) -> ORBSLAM3Wrapper:
    s = cfg.get("slam", {})
    return ORBSLAM3Wrapper(
        vocab_path=s.get("vocab_path", "/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt"),
        settings_path=s.get("settings_path", "config/stereo_imu_settings.yaml"),
        use_imu=s.get("use_imu", False),
        trajectory_maxlen=s.get("max_trajectory_points", 1000),
        frame_skip=s.get("skip_frames", 1),
    )


def _build_publisher(cfg: Dict[str, Any]) -> AWSIoTPublisher:
    a = cfg.get("aws", {})
    return AWSIoTPublisher(
        endpoint=a.get("endpoint", ""),
        thing_name=a.get("thing_name", "jetson-robot-01"),
        cert_path=a.get("certificate", "certs/certificate.pem.crt"),
        key_path=a.get("private_key", "certs/private.pem.key"),
        root_ca=a.get("root_ca"),
        client_id=a.get("client_id"),
        publish_interval=a.get("publish_interval", 5),
        topic_prefix=a.get("topic_prefix", "robot"),
    )


def _init_slam_with_retry(slam: ORBSLAM3Wrapper) -> None:
    """Try to initialise SLAM up to ``_SLAM_INIT_RETRIES`` times.

    On repeated failure the wrapper still works — it runs its internal
    mock backend.
    """
    for attempt in range(1, _SLAM_INIT_RETRIES + 1):
        try:
            slam.initialise()
            return
        except FileNotFoundError as exc:
            logger.error(
                "SLAM init attempt %d/%d failed: %s",
                attempt, _SLAM_INIT_RETRIES, exc,
            )
            if attempt < _SLAM_INIT_RETRIES:
                time.sleep(_SLAM_INIT_DELAY)

    # Final attempt — will fall back to mock inside the wrapper.
    logger.warning("All SLAM init retries exhausted -- starting in mock mode")
    slam.initialise()


# ---------------------------------------------------------------------------
# Drain helpers
# ---------------------------------------------------------------------------

def _drain_imu(camera: CameraIMUHandler, max_samples: int = 10) -> List[IMUSample]:
    """Collect up to *max_samples* IMU readings without blocking."""
    samples: List[IMUSample] = []
    for _ in range(max_samples):
        try:
            s = camera.imu_queue.get_nowait()
            samples.append(s)
        except Empty:
            break
    return samples


# ---------------------------------------------------------------------------
# Visualisation
# ---------------------------------------------------------------------------

def _show_stereo(
    left: np.ndarray,
    right: np.ndarray,
    state: TrackingState,
) -> bool:
    """Display a side-by-side stereo view.  Returns ``False`` if the
    user presses 'q' to quit.
    """
    try:
        import cv2
    except ImportError:
        return True

    combined = np.hstack((left, right))
    colour = (0, 255, 0) if state == TrackingState.TRACKING else (0, 0, 255)
    cv2.putText(
        combined, state.name, (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2,
    )
    cv2.imshow("Stereo", combined)
    return (cv2.waitKey(1) & 0xFF) != ord("q")


# ---------------------------------------------------------------------------
# IMU debug display
# ---------------------------------------------------------------------------

def _print_imu_debug(imu_batch: List[Any], pos: Optional[tuple]) -> None:
    """Print a single overwriting line with accel, tilt, gyro, and position.

    Uses the most recent sample in *imu_batch*.
    Tilt is computed from the accelerometer assuming Y is the gravity axis:
      roll  = angle of X relative to gravity (sideways tilt)
      pitch = angle of Z relative to gravity (forward/back tilt)
    """
    if not imu_batch:
        return

    s = imu_batch[-1]
    ax = float(s.get("accel_x", 0.0))
    ay = float(s.get("accel_y", 0.0))
    az = float(s.get("accel_z", 0.0))
    gx = float(s.get("gyro_x", 0.0))
    gy = float(s.get("gyro_y", 0.0))
    gz = float(s.get("gyro_z", 0.0))

    # Tilt angles from accelerometer (degrees)
    roll  = math.degrees(math.atan2(ax, ay))
    pitch = math.degrees(math.atan2(az, ay))

    pos_str = ""
    if pos is not None:
        pos_str = f"  dr=({pos[0]:+.3f},{pos[2]:+.3f})m"

    line = (
        f"[IMU]  "
        f"accel x={ax:+.3f}g y={ay:+.3f}g z={az:+.3f}g  |  "
        f"tilt roll={roll:+.1f}° pitch={pitch:+.1f}°  |  "
        f"gyro x={gx:+.1f} y={gy:+.1f} z={gz:+.1f} °/s"
        f"{pos_str}"
    )
    # Pad to clear any leftover characters, then overwrite
    print(f"\r{line:<120}", end="", flush=True)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    args = _parse_args()
    cfg = _load_config(args.config)
    _setup_logging(cfg, args.verbose)

    logger.info("Starting Jetson robot SLAM pipeline")

    # ---- build components ------------------------------------------------ #
    if args.sim:
        logger.info(
            "Simulation mode -- using Isaac Sim via ROS 2  "
            "(left=%s  right=%s  imu=%s)",
            args.sim_left_topic, args.sim_right_topic, args.sim_imu_topic,
        )
        camera = _build_sim_camera(
            cfg,
            args.sim_left_topic,
            args.sim_right_topic,
            args.sim_imu_topic,
        )
    else:
        camera = _build_camera(cfg, mock=args.mock, enable_stereo=(args.mode != "imu_only"))

    slam: Optional[ORBSLAM3Wrapper] = None
    if args.mode != "imu_only":
        slam = _build_slam(cfg)
    else:
        logger.info("Mode: imu_only — SLAM disabled, publishing IMU data only")

    publisher: Optional[AWSIoTPublisher] = None
    aws_cfg = cfg.get("aws", {})
    if not args.no_aws and aws_cfg.get("enabled", True):
        publisher = _build_publisher(cfg)

    perf_interval = cfg.get("performance", {}).get("log_stats_interval", 30)
    perf = _PerfMonitor(interval=perf_interval)

    # ---- telemetry file -------------------------------------------------- #
    if not args.telemetry_file:
        thing_name = cfg.get("aws", {}).get("thing_name", "robot")
        run_ts = datetime.datetime.now().strftime("%Y-%m-%dT%H-%M-%S")
        args.telemetry_file = f"telemetry/{thing_name}_{run_ts}.jsonl"
    telem_path = Path(args.telemetry_file)
    if not telem_path.is_absolute():
        telem_path = _PROJECT_ROOT / telem_path
    telem_path.parent.mkdir(parents=True, exist_ok=True)
    telem_file = open(telem_path, "a")
    logger.info("Telemetry file: %s", telem_path)

    # ---- graceful shutdown ----------------------------------------------- #
    shutdown = False

    def _on_signal(sig: int, _: Any) -> None:
        nonlocal shutdown
        logger.info("Received signal %d -- shutting down", sig)
        shutdown = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    # ---- start ----------------------------------------------------------- #
    # Initialise SLAM first: if the C++ System() constructor crashes it
    # starts/leaks internal threads that corrupt the process.  Starting SLAM
    # before the cameras ensures a clean fallback to mock mode without
    # affecting GStreamer / Argus camera capture.
    if slam is not None:
        _init_slam_with_retry(slam)
        # If SLAM fell back to mock (bindings missing or init failure), disable
        # it entirely so fake positions never reach the database.
        if slam._mock:
            logger.warning(
                "SLAM unavailable (bindings missing or init failed) -- "
                "disabling SLAM, will use IMU dead reckoning"
            )
            slam = None

    try:
        camera.start()
    except RuntimeError:
        logger.exception("Camera initialisation failed -- aborting")
        sys.exit(1)

    if publisher is not None:
        publisher.connect()

    logger.info("All systems running -- press Ctrl+C to stop")

    # ---- run loop -------------------------------------------------------- #
    visualize = args.visualize
    last_status_publish = 0.0
    _STATUS_INTERVAL = 10.0  # seconds between sensor status publishes

    imu_only_mode = (args.mode == "imu_only")
    # Also enable dead reckoning when SLAM was disabled due to init failure.
    dead_reckoning = DeadReckoningIntegrator() if (imu_only_mode or slam is None) else None

    try:
        while not shutdown:
            if imu_only_mode:
                # IMU-only: no camera frames needed — block on IMU queue directly.
                imu_sample = camera.get_imu_sample(timeout=0.5)
                if imu_sample is None:
                    continue
                ts = imu_sample.get("timestamp", time.monotonic())
                imu_batch = [imu_sample] + _drain_imu(camera, max_samples=9)
                left = right = None
            else:
                # 1. Get stereo frame (blocks up to 0.5 s).
                frame = camera.get_stereo_frame(timeout=0.5)
                if frame is None:
                    continue
                left, right, ts = frame
                # 2. Collect recent IMU measurements.
                imu_batch = _drain_imu(camera, max_samples=10)

            perf.tick_frame()

            # 3. Process through SLAM (skipped in imu_only mode).
            pose = None
            if slam is not None:
                pose = slam.process_frame(
                    left, right, ts,
                    imu_batch if imu_batch else None,
                )
            tracking_ok = pose is not None
            if slam is not None:
                perf.tick_slam(tracking_ok)

            # 4. Publish to AWS (only if connected) and save locally.
            thing_name = cfg.get("aws", {}).get("thing_name", "jetson-robot-01")
            if tracking_ok:
                if publisher is not None and publisher.is_connected:
                    publisher.publish_pose(pose, ts)
                payload = AWSIoTPublisher.build_pose_payload(
                    pose, ts, thing_name,
                )
                telem_file.write(json.dumps(payload) + "\n")
                telem_file.flush()
            elif imu_batch:
                # SLAM unavailable or disabled — publish IMU-only data.
                logger.debug("SLAM unavailable — publishing IMU-only data")
                pos = dead_reckoning.update(imu_batch) if dead_reckoning else None
                if publisher is not None and publisher.is_connected:
                    publisher.publish_imu(imu_batch, ts, position=pos)
                payload = AWSIoTPublisher.build_imu_payload(
                    imu_batch, ts, thing_name, position=pos,
                )
                telem_file.write(json.dumps(payload) + "\n")
                telem_file.flush()

            # 5. Optional IMU debug display.
            if args.debug_imu and imu_batch:
                _print_imu_debug(imu_batch, pos if not tracking_ok else None)

            # 6. Optional visualisation (only when camera frames are available).
            if visualize and not imu_only_mode:
                state = slam.state if slam is not None else TrackingState.LOST
                if not _show_stereo(left, right, state):
                    shutdown = True

            # 6. Periodic performance report.
            perf.maybe_report(
                camera.frame_queue.qsize(),
                camera.imu_queue.qsize(),
                slam,
                publisher,
            )

            # 7. Periodic sensor status publish.
            now = time.monotonic()
            if now - last_status_publish >= _STATUS_INTERVAL:
                imu_mock = (
                    camera._imu_reader is not None
                    and camera._imu_reader._mock
                )
                is_sim = isinstance(camera, IsaacSimCameraIMUHandler)
                sensors = {
                    "camera": "n/a" if imu_only_mode else ("sim" if is_sim else ("mock" if camera._mock else ("ok" if camera.is_running else "error"))),
                    "imu": "sim" if is_sim else ("mock" if imu_mock else "ok"),
                    "slam": "disabled" if slam is None else ("mock" if slam._mock else "ok"),
                    "gps": "n/a",
                }
                if publisher is not None:
                    publisher.publish_sensor_status(sensors)
                # Write sensor status locally for the dashboard server
                status_payload = {
                    "device_id": cfg.get("aws", {}).get("thing_name", "jetson-robot-01"),
                    "timestamp": round(time.time(), 4),
                    "sensors": sensors,
                }
                status_path = telem_path.parent / "sensor_status.json"
                try:
                    with open(status_path, "w") as sf:
                        json.dump(status_payload, sf)
                except OSError:
                    logger.debug("Failed to write sensor_status.json")
                last_status_publish = now

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received")

    # ---- shutdown -------------------------------------------------------- #
    finally:
        logger.info("Shutting down...")
        camera.stop()

        if slam is not None:
            traj_path = str(_PROJECT_ROOT / "trajectory.txt")
            slam.save_trajectory(traj_path)
            slam.shutdown()

        if publisher is not None:
            publisher.disconnect()

        telem_file.close()
        logger.info("Telemetry file closed: %s", telem_path)

        if visualize:
            try:
                import cv2
                cv2.destroyAllWindows()
            except Exception:
                pass

    logger.info("Goodbye")


if __name__ == "__main__":
    main()
