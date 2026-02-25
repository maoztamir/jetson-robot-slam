#!/usr/bin/env python3
# Preload libgomp before any other import to avoid aarch64 TLS allocation error.
# Must be the very first executable line in the file.
import ctypes, ctypes.util  # noqa: E401
for _lib in ("/usr/lib/aarch64-linux-gnu/libgomp.so.1", "gomp"):
    try:
        ctypes.CDLL(_lib, mode=ctypes.RTLD_GLOBAL)
        break
    except OSError:
        pass

"""Verify ORB_SLAM3 installation and Python binding on the current device.

Checks:
  1. orbslam3 Python binding can be imported
  2. ORB_SLAM3 System initialises without crashing (vocab + settings)
  3. A single stereo frame can be tracked
  4. System shuts down cleanly

Usage:
    python scripts/verify_slam.py
    python scripts/verify_slam.py --mock          # skip live cameras
    python scripts/verify_slam.py --no-camera     # skip frame-tracking test
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_PROJECT_ROOT))

_VOCAB    = "/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt"
_SETTINGS = str(_PROJECT_ROOT / "config/stereo_imu_settings.yaml")

PASS = "[PASS]"
FAIL = "[FAIL]"
SKIP = "[SKIP]"


def check_import() -> bool:
    print("1. Importing orbslam3 binding ...", end=" ", flush=True)
    try:
        import orbslam3  # noqa: F401
        print(PASS)
        return True
    except ImportError as e:
        print(f"{FAIL}  {e}")
        return False


def check_init(vocab: str, settings: str) -> bool:
    print("2. Initialising ORB_SLAM3 System ...", end=" ", flush=True)
    try:
        import orbslam3
        slam = orbslam3.System(vocab, settings, orbslam3.Sensor.STEREO, False)
        slam.shutdown()
        print(PASS)
        return True
    except Exception as e:
        print(f"{FAIL}  {e}")
        return False


def check_track(vocab: str, settings: str, mock: bool) -> bool:
    print("3. Tracking one stereo frame ...", end=" ", flush=True)
    try:
        import cv2
        import numpy as np
        import orbslam3
        from src.sensors.camera_imu_handler import CameraIMUHandler

        slam = orbslam3.System(vocab, settings, orbslam3.Sensor.STEREO, False)

        camera = CameraIMUHandler(width=640, height=480, fps=30,
                                  enable_imu=False, mock=mock)
        camera.start()

        frame = None
        for _ in range(30):          # wait up to ~1 s for first frame
            frame = camera.get_stereo_frame(timeout=0.1)
            if frame is not None:
                break

        camera.stop()

        if frame is None:
            slam.shutdown()
            print(f"{FAIL}  no frame received from camera")
            return False

        left, right, ts = frame
        pose = slam.track_stereo(left, right, ts)
        slam.shutdown()

        if pose is not None:
            print(f"{PASS}  pose shape={pose.shape}")
        else:
            print(f"{PASS}  pose=None (SLAM not yet initialised -- normal for frame 1)")
        return True

    except Exception as e:
        print(f"{FAIL}  {e}")
        return False


def main() -> None:
    p = argparse.ArgumentParser(description="Verify ORB_SLAM3 installation")
    p.add_argument("--vocab",      default=_VOCAB)
    p.add_argument("--settings",   default=_SETTINGS)
    p.add_argument("--mock",       action="store_true",
                   help="Use synthetic cameras instead of hardware")
    p.add_argument("--no-camera",  action="store_true",
                   help="Skip the camera + frame-tracking test")
    args = p.parse_args()

    print("=" * 55)
    print("  ORB_SLAM3 verification")
    print(f"  vocab    : {args.vocab}")
    print(f"  settings : {args.settings}")
    print("=" * 55)

    results = []

    results.append(check_import())
    if not results[-1]:
        print("\nCannot continue without the binding -- aborting.")
        sys.exit(1)

    results.append(check_init(args.vocab, args.settings))

    if args.no_camera:
        print("3. Tracking one stereo frame ... " + SKIP + "  (--no-camera)")
    else:
        results.append(check_track(args.vocab, args.settings, args.mock))

    print("=" * 55)
    passed = all(r for r in results)
    print("  Result:", PASS + "  all checks passed" if passed
          else FAIL + "  one or more checks failed")
    print("=" * 55)
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
