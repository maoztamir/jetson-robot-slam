#!/bin/bash
# Build OpenCV 4.10.0 for Python 3.8 (pyenv) on Raspberry Pi 5
# with GStreamer + libcamera support. No CUDA.
# Runtime: ~1 hour on RPi 5 with -j4.

set -e

OPENCV_VERSION="4.10.0"
BUILD_DIR="/tmp/opencv_build"

# ── Locate Python 3.8 from pyenv ─────────────────────────────────────────────
PYTHON38=$(pyenv which python3.8 2>/dev/null || which python3.8)
PYTHON38_PREFIX=$(python3.8 -c "import sys; print(sys.prefix)")
PYTHON38_INC=$(python3.8 -c "from sysconfig import get_path; print(get_path('include'))")
PYTHON38_LIB=$(find "$PYTHON38_PREFIX/lib" -name "libpython3.8*.so*" | head -1)
NUMPY_INC=$(python3.8 -c "import numpy; print(numpy.get_include())")
SITE_PACKAGES=$(python3.8 -c "from sysconfig import get_path; print(get_path('purelib'))")

echo "Python  : $PYTHON38"
echo "Include : $PYTHON38_INC"
echo "Library : $PYTHON38_LIB"
echo "NumPy   : $NUMPY_INC"
echo "Install : $SITE_PACKAGES"

# ── Install build dependencies ────────────────────────────────────────────────
sudo apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-libcamera \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev v4l-utils \
    libxvidcore-dev libx264-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libeigen3-dev \
    libatlas-base-dev \
    gfortran \
    openexr \
    python3.8-dev

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
    -DWITH_CUDA=OFF \
    -DWITH_V4L=ON \
    -DWITH_LIBV4L=ON \
    -DBUILD_opencv_python2=OFF \
    -DBUILD_opencv_python3=ON \
    -DPYTHON3_EXECUTABLE="$PYTHON38" \
    -DPYTHON3_INCLUDE_DIR="$PYTHON38_INC" \
    -DPYTHON3_LIBRARY="$PYTHON38_LIB" \
    -DPYTHON3_NUMPY_INCLUDE_DIRS="$NUMPY_INC" \
    -DOPENCV_PYTHON3_INSTALL_PATH="$SITE_PACKAGES" \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DINSTALL_PYTHON_EXAMPLES=OFF

# ── Build ─────────────────────────────────────────────────────────────────────
make -j4

sudo make install
sudo ldconfig

echo ""
echo "✓ OpenCV $OPENCV_VERSION installed for Python 3.8"
echo ""
echo "Verify with:"
echo "  python3.8 -c \"import cv2; print(cv2.__version__); print([l for l in cv2.getBuildInformation().splitlines() if 'GStreamer' in l])\""
