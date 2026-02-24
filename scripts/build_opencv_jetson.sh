#!/bin/bash
# Build OpenCV 4.5.5 for Python 3.8 (pyenv) on Jetson Nano
# with GStreamer + CUDA support.
# Runtime: ~1-2 hours. Run from any directory.

set -e

OPENCV_VERSION="4.5.5"
BUILD_DIR="/tmp/opencv_build"

# ── Locate Python 3.8 from pyenv ─────────────────────────────────────────────
PYTHON38=$(pyenv which python3.8 2>/dev/null || which python3.8)
PYTHON38_PREFIX=$(python3.8 -c "import sys; print(sys.prefix)")
PYTHON38_INC=$(python3.8 -c "from sysconfig import get_path; print(get_path('include'))")
PYTHON38_LIB=$(find "$PYTHON38_PREFIX/lib" -name "libpython3.8*.so*" | head -1)
NUMPY_INC=$(python3.8 -c "import numpy; print(numpy.get_include())")

echo "Python  : $PYTHON38"
echo "Include : $PYTHON38_INC"
echo "Library : $PYTHON38_LIB"
echo "NumPy   : $NUMPY_INC"

# ── Install build dependencies ────────────────────────────────────────────────
sudo apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev v4l-utils \
    libxvidcore-dev libx264-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libeigen3-dev \
    gfortran \
    openexr \
    libatlas-base-dev

# ── Clone sources ─────────────────────────────────────────────────────────────
mkdir -p "$BUILD_DIR" && cd "$BUILD_DIR"

[ -d opencv ]         || git clone --depth 1 --branch "$OPENCV_VERSION" https://github.com/opencv/opencv.git
[ -d opencv_contrib ] || git clone --depth 1 --branch "$OPENCV_VERSION" https://github.com/opencv/opencv_contrib.git

mkdir -p opencv/build && cd opencv/build

# ── Configure ─────────────────────────────────────────────────────────────────
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DOPENCV_EXTRA_MODULES_PATH="$BUILD_DIR/opencv_contrib/modules" \
    -DWITH_GSTREAMER=ON \
    -DWITH_CUDA=ON \
    -DWITH_CUDNN=ON \
    -DCUDA_ARCH_BIN="5.3" \
    -DENABLE_FAST_MATH=ON \
    -DCUDA_FAST_MATH=ON \
    -DWITH_CUBLAS=ON \
    -DBUILD_opencv_python2=OFF \
    -DBUILD_opencv_python3=ON \
    -DPYTHON3_EXECUTABLE="$PYTHON38" \
    -DPYTHON3_INCLUDE_DIR="$PYTHON38_INC" \
    -DPYTHON3_LIBRARY="$PYTHON38_LIB" \
    -DPYTHON3_NUMPY_INCLUDE_DIRS="$NUMPY_INC" \
    -DOPENCV_PYTHON3_INSTALL_PATH="$PYTHON38_PREFIX/lib/python3.8/site-packages" \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DINSTALL_PYTHON_EXAMPLES=OFF

# ── Build (use -j3 to leave one core free and avoid OOM) ─────────────────────
make -j3

sudo make install
sudo ldconfig

echo ""
echo "✓ OpenCV $OPENCV_VERSION installed for Python 3.8"
echo "  Verify with:"
echo "  python3.8 -c \"import cv2; print(cv2.__version__); print([l for l in cv2.getBuildInformation().splitlines() if 'GStreamer' in l])\""
