#!/usr/bin/env python3
"""Test libcamerasrc GStreamer pipelines for both IMX219 cameras on RPi 5.

Tries multiple format options to find which one the PiSP ISP supports.
"""

import cv2

CAM0_NAME = None  # default (first camera = i2c@88000 = CAM0/left)
CAM1_NAME = "/base/axi/pcie@1000120000/rp1/i2c@80000/imx219@10"

WIDTH  = 640
HEIGHT = 480
FPS    = 30

# Candidate pipelines to try in order.  libcamerasrc on RPi5 PiSP ISP can
# output BGR, RGB, NV12 etc. depending on the libcamera / GStreamer version.
PIPELINE_TEMPLATES = [
    # Force BGR directly — skips videoconvert
    (
        "BGR-direct",
        "libcamerasrc {name}! "
        "video/x-raw, format=BGR, width={w}, height={h}, framerate={fps}/1 ! "
        "appsink sync=false drop=true"
    ),
    # RGB then convert
    (
        "RGB+convert",
        "libcamerasrc {name}! "
        "video/x-raw, format=RGB, width={w}, height={h}, framerate={fps}/1 ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink sync=false drop=true"
    ),
    # NV12 then convert (common ISP output)
    (
        "NV12+convert",
        "libcamerasrc {name}! "
        "video/x-raw, format=NV12, width={w}, height={h}, framerate={fps}/1 ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink sync=false drop=true"
    ),
    # BGRx then convert
    (
        "BGRx+convert",
        "libcamerasrc {name}! "
        "video/x-raw, format=BGRx, width={w}, height={h}, framerate={fps}/1 ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink sync=false drop=true"
    ),
    # No format constraint — let libcamerasrc negotiate, convert at end
    (
        "auto+convert",
        "libcamerasrc {name}! "
        "video/x-raw, width={w}, height={h}, framerate={fps}/1 ! "
        "videoconvert ! video/x-raw, format=BGR ! "
        "appsink sync=false drop=true"
    ),
]


def try_pipeline(label, pipeline):
    print(f"  [{label}]  {pipeline}")
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print(f"  [{label}]  FAIL — could not open pipeline")
        return False
    ok, frame = cap.read()
    cap.release()
    if ok and frame is not None:
        print(f"  [{label}]  PASS — frame shape: {frame.shape}")
        return True
    print(f"  [{label}]  FAIL — cap.read() returned False")
    return False


def find_working_pipeline(camera_label, camera_name=None):
    name_prop = f'camera-name="{camera_name}" ' if camera_name else ""
    print(f"\n{'='*60}")
    print(f"{camera_label}")
    print(f"{'='*60}")
    for label, template in PIPELINE_TEMPLATES:
        pipe = template.format(name=name_prop, w=WIDTH, h=HEIGHT, fps=FPS)
        if try_pipeline(label, pipe):
            out = f"/tmp/{camera_label.lower().replace(' ', '_')}.jpg"
            cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
            ok, frame = cap.read()
            cap.release()
            if ok:
                cv2.imwrite(out, frame)
                print(f"\n  Saved test frame to {out}")
            print(f"\n  Working pipeline:\n  {pipe}")
            return pipe
    print(f"\n  No working pipeline found for {camera_label}")
    return None


if __name__ == "__main__":
    p0 = find_working_pipeline("CAM0 (left)",  CAM0_NAME)
    p1 = find_working_pipeline("CAM1 (right)", CAM1_NAME)

    print(f"\n{'='*60}")
    print(f"CAM0 (left) : {'PASS' if p0 else 'FAIL'}")
    print(f"CAM1 (right): {'PASS' if p1 else 'FAIL'}")
    print(f"{'='*60}")

    if p0 and p1:
        print("\nBoth cameras OK.")
        print("\nAdd to config/local_config.yaml:")
        print("  camera:")
        print("    backend: rpi5")
