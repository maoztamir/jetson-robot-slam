#!/usr/bin/env python3
"""Test libcamerasrc GStreamer pipelines for both IMX219 cameras on RPi 5."""

import cv2

CAM0_NAME = None  # default (first camera)
CAM1_NAME = "/base/axi/pcie@1000120000/rp1/i2c@80000/imx219@10"

WIDTH  = 640
HEIGHT = 480
FPS    = 30


def build_pipeline(camera_name=None):
    name_prop = f'camera-name="{camera_name}" ' if camera_name else ""
    return (
        f"libcamerasrc {name_prop}! "
        f"video/x-raw, width={WIDTH}, height={HEIGHT}, framerate={FPS}/1 ! "
        f"videoconvert ! video/x-raw, format=BGR ! "
        f"appsink sync=false drop=true"
    )


def test_camera(label, camera_name=None):
    pipe = build_pipeline(camera_name)
    print(f"\n[{label}] Pipeline:\n  {pipe}")

    cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print(f"[{label}] FAILED -- VideoCapture could not open pipeline")
        return False

    ok, frame = cap.read()
    cap.release()

    if ok:
        print(f"[{label}] OK -- frame shape: {frame.shape}  dtype: {frame.dtype}")
        out = f"/tmp/{label.lower().replace(' ', '_')}.jpg"
        cv2.imwrite(out, frame)
        print(f"[{label}] Saved test frame to {out}")
        return True
    else:
        print(f"[{label}] FAILED -- cap.read() returned False")
        return False


if __name__ == "__main__":
    print("=" * 60)
    print("RPi 5 camera pipeline test")
    print("=" * 60)

    r0 = test_camera("CAM0 (left)",  CAM0_NAME)
    r1 = test_camera("CAM1 (right)", CAM1_NAME)

    print("\n" + "=" * 60)
    print(f"CAM0 (left) : {'PASS' if r0 else 'FAIL'}")
    print(f"CAM1 (right): {'PASS' if r1 else 'FAIL'}")
    print("=" * 60)

    if r0 and r1:
        print("\nBoth cameras OK -- pipeline is ready.")
    else:
        print("\nOne or more cameras failed. Check GStreamer logs above.")
