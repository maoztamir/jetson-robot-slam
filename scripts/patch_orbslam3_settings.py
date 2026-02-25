#!/usr/bin/env python3
"""Patch ORB_SLAM3/src/Settings.cc to catch precomputeRectificationMaps failures.

On aarch64 (Jetson) the cv::stereoRectify call inside
precomputeRectificationMaps() can throw a cv::Exception due to ARM
floating-point behaviour with certain camera parameters.  This patch
wraps the call in a try/catch so ORB_SLAM3 initialises gracefully
instead of aborting.

Run once (with sudo) after cloning / updating ORB_SLAM3:

    sudo python3 scripts/patch_orbslam3_settings.py
    cd /opt/ORB_SLAM3 && sudo ./build.sh

"""

import sys
from pathlib import Path

SETTINGS_CC = Path("/opt/ORB_SLAM3/src/Settings.cc")

OLD = (
    "        if(bNeedToRectify_){\n"
    "            precomputeRectificationMaps();\n"
    '            cout << "\\t-Computed rectification maps" << endl;\n'
    "        }"
)

NEW = (
    "        if(bNeedToRectify_){\n"
    "            try {\n"
    "                precomputeRectificationMaps();\n"
    '                cout << "\\t-Computed rectification maps" << endl;\n'
    "            } catch(const cv::Exception& e) {\n"
    '                std::cerr << "Warning: precomputeRectificationMaps failed: "\n'
    '                          << e.what() << " -- skipping." << std::endl;\n'
    "                bNeedToRectify_ = false;\n"
    "            }\n"
    "        }"
)

ALREADY_PATCHED_MARKER = "Warning: precomputeRectificationMaps failed"


def main() -> None:
    if not SETTINGS_CC.exists():
        print(f"ERROR: {SETTINGS_CC} not found -- is ORB_SLAM3 installed?")
        sys.exit(1)

    content = SETTINGS_CC.read_text()

    if ALREADY_PATCHED_MARKER in content:
        print("Already patched -- nothing to do.")
        sys.exit(0)

    if OLD not in content:
        print("ERROR: expected pattern not found in Settings.cc.")
        print("The file may have been modified differently than expected.")
        print("Look for the 'if(bNeedToRectify_)' block and apply manually.")
        sys.exit(1)

    patched = content.replace(OLD, NEW, 1)
    SETTINGS_CC.write_text(patched)
    print(f"Patch applied to {SETTINGS_CC}")
    print()
    print("Next steps:")
    print("  cd /opt/ORB_SLAM3 && sudo ./build.sh")
    print("  cd ~/jetson-robot-slam/bindings && rm -rf build && mkdir build && cd build")
    print("  cmake .. -DPYTHON_EXECUTABLE=$(which python3) && make -j$(nproc)")
    print("  cp orbslam3*.so ~/jetson-robot-slam/")


if __name__ == "__main__":
    main()
