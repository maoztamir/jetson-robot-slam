#!/usr/bin/env python3
# Preload libgomp before any other import to avoid aarch64 TLS allocation error.
import ctypes, ctypes.util  # noqa: E401
for _lib in ("/usr/lib/aarch64-linux-gnu/libgomp.so.1", "gomp"):
    try:
        ctypes.CDLL(_lib, mode=ctypes.RTLD_GLOBAL)
        break
    except OSError:
        pass

"""Stereo SLAM visualiser / recorder.

Records a video showing whether SLAM is actually tracking:

  Left panel  – left camera feed with ORB keypoints drawn in green
  Right panel – top-down 2-D trajectory map
                  cyan dots  = 3-D map points (real SLAM only)
                  white line = camera path
                  red circle = current position

The script works even while ORB_SLAM3 bindings are broken: it falls
back to the mock backend automatically and still records the figure-8
synthetic trajectory so you can verify the pipeline end-to-end.

Usage (on the Jetson, venv active):

    python scripts/slam_recorder.py                    # hardware cameras
    python scripts/slam_recorder.py --mock             # synthetic cameras
    python scripts/slam_recorder.py --out test.avi --duration 30
    python scripts/slam_recorder.py --no-slam          # features only

Output file: slam_recording.avi (640+640 x 480 pixels).
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_PROJECT_ROOT))

from src.sensors.camera_imu_handler import CameraIMUHandler
from src.slam.orb_slam3_wrapper import ORBSLAM3Wrapper

logger = logging.getLogger("recorder")

# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------

_CAM_W, _CAM_H = 640, 480   # each camera panel
_MAP_W, _MAP_H = 640, 480   # trajectory panel
_OUT_W = _CAM_W + _MAP_W    # total output width  = 1280
_OUT_H = _CAM_H             # total output height = 480

# ---------------------------------------------------------------------------
# ORB feature detector (for visual feedback independent of SLAM internals)
# ---------------------------------------------------------------------------

_ORB_DETECTOR = cv2.ORB_create(nfeatures=500)


def _draw_keypoints(img: np.ndarray) -> np.ndarray:
    """Return a copy of *img* with ORB keypoints drawn as green circles."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    kps = _ORB_DETECTOR.detect(gray, None)
    out = img.copy()
    cv2.drawKeypoints(
        out, kps, out,
        color=(0, 255, 0),
        flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
    )
    return out


# ---------------------------------------------------------------------------
# Trajectory canvas
# ---------------------------------------------------------------------------

class _TrajectoryMap:
    """Running 2-D bird's-eye-view trajectory canvas (XZ plane)."""

    _SCALE = 50.0          # pixels per metre
    _MAX_MAP_PTS = 3000

    def __init__(self) -> None:
        self._origin = np.array([_MAP_W // 2, _MAP_H // 2], dtype=np.float32)
        self._path: List[Tuple[int, int]] = []
        self._map_pts: List[Tuple[float, float, float]] = []

    # world coords → canvas pixel
    def _w2p(self, x: float, z: float) -> Tuple[int, int]:
        px = int(self._origin[0] + x * self._SCALE)
        py = int(self._origin[1] - z * self._SCALE)   # Z forward = up
        return px, py

    def update(
        self,
        pose: Optional[np.ndarray],
        map_pts: Optional[np.ndarray],
    ) -> None:
        if pose is not None:
            x, z = float(pose[0, 3]), float(pose[2, 3])
            self._path.append(self._w2p(x, z))

        if map_pts is not None and len(map_pts):
            new = [(float(p[0]), float(p[1]), float(p[2])) for p in map_pts]
            self._map_pts = (self._map_pts + new)[-self._MAX_MAP_PTS:]

    def render(self, state: str, backend: str) -> np.ndarray:
        canvas = np.zeros((_MAP_H, _MAP_W, 3), dtype=np.uint8)

        # subtle grid
        step = int(self._SCALE)
        for x in range(0, _MAP_W, step):
            cv2.line(canvas, (x, 0), (x, _MAP_H), (28, 28, 28), 1)
        for y in range(0, _MAP_H, step):
            cv2.line(canvas, (0, y), (_MAP_W, y), (28, 28, 28), 1)

        # origin cross
        ox, oy = int(self._origin[0]), int(self._origin[1])
        cv2.line(canvas, (ox - 10, oy), (ox + 10, oy), (60, 60, 60), 1)
        cv2.line(canvas, (ox, oy - 10), (ox, oy + 10), (60, 60, 60), 1)

        # 3-D map points projected to XZ plane (cyan)
        for mp in self._map_pts:
            px, py = self._w2p(mp[0], mp[2])
            if 0 <= px < _MAP_W and 0 <= py < _MAP_H:
                cv2.circle(canvas, (px, py), 1, (200, 200, 0), -1)

        # trajectory path (white)
        if len(self._path) > 1:
            pts = np.array(self._path, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(canvas, [pts], False, (180, 180, 180), 1)

        # current position (red)
        if self._path:
            cx, cy = self._path[-1]
            if 0 <= cx < _MAP_W and 0 <= cy < _MAP_H:
                cv2.circle(canvas, (cx, cy), 6, (0, 0, 255), -1)
                cv2.circle(canvas, (cx, cy), 6, (255, 255, 255), 1)

        # HUD text
        bc = (0, 165, 255) if backend == "MOCK" else (0, 220, 0)
        cv2.putText(canvas, f"[{backend}] {state}", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, bc, 1)
        cv2.putText(canvas, f"poses: {len(self._path)}", (8, 44),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
        cv2.putText(canvas, f"map pts: {len(self._map_pts)}", (8, 64),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
        cv2.putText(canvas, "top-down  (X right, Z forward)",
                    (8, _MAP_H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (80, 80, 80), 1)
        return canvas


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Record stereo SLAM visualisation to a video file",
    )
    p.add_argument("--out", default="slam_recording.avi",
                   help="Output video path (default: slam_recording.avi)")
    p.add_argument("--fps", type=float, default=15.0,
                   help="Output video FPS (default: 15)")
    p.add_argument("--duration", type=float, default=0.0,
                   help="Stop after N seconds; 0 = until Ctrl-C")
    p.add_argument("--mock", action="store_true",
                   help="Use synthetic cameras (no hardware required)")
    p.add_argument("--no-slam", action="store_true",
                   help="Skip SLAM; record camera feed + features only")
    p.add_argument("--vocab",
                   default="/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt")
    p.add_argument("--settings",
                   default=str(_PROJECT_ROOT / "config/stereo_imu_settings.yaml"))
    p.add_argument("--verbose", action="store_true")
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    out_path = Path(args.out)
    if not out_path.is_absolute():
        out_path = _PROJECT_ROOT / out_path
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # -- SLAM init (before cameras to avoid leaked-thread corruption) -------
    slam: Optional[ORBSLAM3Wrapper] = None
    if not args.no_slam:
        slam = ORBSLAM3Wrapper(
            vocab_path=args.vocab,
            settings_path=args.settings,
            use_imu=False,
        )
        slam.initialise()
        backend = "MOCK" if slam._mock else "ORB_SLAM3"
        logger.info("SLAM backend: %s", backend)
    else:
        backend = "NONE"

    # -- Camera -------------------------------------------------------------
    camera = CameraIMUHandler(
        width=_CAM_W, height=_CAM_H, fps=30,
        enable_imu=False,
        mock=args.mock,
    )
    camera.start()

    # -- Video writer -------------------------------------------------------
    fourcc = cv2.VideoWriter_fourcc(*"XVID")
    writer = cv2.VideoWriter(str(out_path), fourcc, args.fps,
                             (_OUT_W, _OUT_H))
    if not writer.isOpened():
        logger.error("Cannot open video writer: %s", out_path)
        camera.stop()
        sys.exit(1)

    logger.info("Recording → %s  (%dx%d @ %.0f fps)",
                out_path, _OUT_W, _OUT_H, args.fps)
    logger.info("Press Ctrl-C to stop.")

    traj = _TrajectoryMap()
    t_start = time.monotonic()
    frames_written = 0
    fps_counter = 0
    fps_display = 0.0
    fps_t = time.monotonic()

    try:
        while True:
            if args.duration > 0 and (time.monotonic() - t_start) >= args.duration:
                logger.info("Duration reached -- stopping")
                break

            frame = camera.get_stereo_frame(timeout=0.5)
            if frame is None:
                continue
            left, right, ts = frame

            # -- SLAM -------------------------------------------------------
            state_name = "NO_SLAM"
            if slam is not None:
                pose = slam.process_frame(left, right, ts)
                state_name = slam.state.name
                map_pts = slam.get_map_points() if not slam._mock else None
                traj.update(pose, map_pts)

            # -- Feature visualisation on left image -----------------------
            left_vis = _draw_keypoints(left)

            # -- FPS overlay -----------------------------------------------
            fps_counter += 1
            now = time.monotonic()
            if now - fps_t >= 1.0:
                fps_display = fps_counter / (now - fps_t)
                fps_counter = 0
                fps_t = now

            cv2.putText(left_vis, f"{fps_display:.1f} fps  {_CAM_W}x{_CAM_H}",
                        (10, _CAM_H - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (160, 160, 160), 1)
            cv2.putText(left_vis, "LEFT + ORB keypoints", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 0), 1)

            # -- Compose and write -----------------------------------------
            traj_vis = traj.render(state_name, backend)
            output_frame = np.hstack([left_vis, traj_vis])
            writer.write(output_frame)
            frames_written += 1

    except KeyboardInterrupt:
        logger.info("Stopped by user")
    finally:
        camera.stop()
        if slam is not None:
            slam.shutdown()
        writer.release()
        elapsed = time.monotonic() - t_start
        logger.info("Saved %d frames → %s  (%.0f s, %.1f fps avg)",
                    frames_written, out_path,
                    elapsed, frames_written / elapsed if elapsed > 0 else 0)


if __name__ == "__main__":
    main()
