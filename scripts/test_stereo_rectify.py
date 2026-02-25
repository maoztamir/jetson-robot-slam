#!/usr/bin/env python3
"""Test whether cv::stereoRectify works in pure C++ on this device.

Compiles and runs a minimal C++ program that calls stereoRectify and
initUndistortRectifyMap with our camera parameters, completely independent
of Python / ORB_SLAM3 / pybind11.

If this test passes but verify_slam.py fails, the crash is a Python-context
issue (FPU mode, TLS, memory layout) rather than an OpenCV parameter issue.

Usage:
    python scripts/test_stereo_rectify.py
"""

import subprocess
import sys
import tempfile
from pathlib import Path

CPP_SOURCE = r"""
#include <opencv2/opencv.hpp>
#include <stdio.h>

int main() {
    double fx = 460.0, cx = 320.0, cy = 240.0;
    cv::Mat K = (cv::Mat_<double>(3,3) << fx,0,cx, 0,fx,cy, 0,0,1);
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
    cv::Size sz(640, 480);
    cv::Mat R12 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t12 = (cv::Mat_<double>(3,1) << 0.083, 0, 0);
    cv::Mat R1, R2, P1, P2, Q;

    printf("stereoRectify ... "); fflush(stdout);
    cv::stereoRectify(K, D, K, D, sz,
                      R12, t12,
                      R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, 1, sz);
    printf("OK\n");

    printf("P1[0,0] = %.4f\n", P1.at<double>(0,0));

    cv::Mat M1l, M2l;
    printf("initUndistortRectifyMap ... "); fflush(stdout);
    cv::initUndistortRectifyMap(K, D, R1,
                                P1.rowRange(0,3).colRange(0,3),
                                sz, CV_32F, M1l, M2l);
    printf("OK  map size: %dx%d\n", M1l.cols, M1l.rows);
    return 0;
}
"""


def main() -> None:
    print("=" * 55)
    print("  stereoRectify C++ standalone test")
    print("=" * 55)

    with tempfile.TemporaryDirectory() as tmp:
        src  = Path(tmp) / "test_rectify.cpp"
        exe  = Path(tmp) / "test_rectify"

        src.write_text(CPP_SOURCE)

        # Compile
        print("Compiling ... ", end="", flush=True)
        flags = subprocess.run(
            ["pkg-config", "--cflags", "--libs", "opencv4"],
            capture_output=True, text=True
        )
        if flags.returncode != 0:
            # fall back to opencv without version suffix
            flags = subprocess.run(
                ["pkg-config", "--cflags", "--libs", "opencv"],
                capture_output=True, text=True
            )
        if flags.returncode != 0:
            print("FAIL -- pkg-config for opencv not found")
            sys.exit(1)

        result = subprocess.run(
            ["g++", str(src), "-o", str(exe)] + flags.stdout.split(),
            capture_output=True, text=True
        )
        if result.returncode != 0:
            print("FAIL")
            print(result.stderr)
            sys.exit(1)
        print("OK")

        # Run
        print("Running C++ binary:")
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        print(result.stdout, end="")
        if result.stderr:
            print(result.stderr, end="")

        if result.returncode != 0:
            print("\n[FAIL]  C++ binary crashed -- OpenCV itself is broken for these params")
            sys.exit(1)
        else:
            print("\n[PASS]  stereoRectify works in pure C++")
            print("        -> crash in verify_slam.py is a Python-context issue")

    print("=" * 55)


if __name__ == "__main__":
    main()
