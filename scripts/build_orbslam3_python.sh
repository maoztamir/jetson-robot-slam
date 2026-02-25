#!/usr/bin/env bash
# Build ORB_SLAM3 pybind11 Python bindings.
#
# Links against the existing /opt/ORB_SLAM3/lib/libORB_SLAM3.so -- no full
# ORB_SLAM3 rebuild needed.  Takes ~5 minutes.
#
# Usage (run from project root with venv active):
#   bash scripts/build_orbslam3_python.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
BINDINGS_DIR="$PROJECT_ROOT/bindings"
BUILD_DIR="$BINDINGS_DIR/build"

# ---------------------------------------------------------------------------
# Detect venv
# ---------------------------------------------------------------------------
if [ -n "$VIRTUAL_ENV" ]; then
    VENV="$VIRTUAL_ENV"
elif [ -d "$HOME/test38" ]; then
    VENV="$HOME/test38"
elif [ -d "$HOME/venv38" ]; then
    VENV="$HOME/venv38"
else
    echo "ERROR: no venv found. Activate test38 or venv38 first." >&2
    exit 1
fi

# shellcheck disable=SC1091
source "$VENV/bin/activate"
PYTHON="$VENV/bin/python3"

echo "Venv   : $VENV"
echo "Python : $($PYTHON --version)"

# ---------------------------------------------------------------------------
# System deps
# ---------------------------------------------------------------------------
sudo apt-get install -y \
    libpython3.8-dev \
    libeigen3-dev \
    build-essential

# Python build deps -- pin pybind11 to 2.5.x which works with CMake 3.10
pip install --quiet "pybind11==2.5.0" numpy cmake

# ---------------------------------------------------------------------------
# Remove the broken pip stub (wrong arch .so)
# ---------------------------------------------------------------------------
pip uninstall orbslam3 -y 2>/dev/null || true

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

PYBIND11_CMAKE=$($PYTHON -m pybind11 --cmakedir)

# Use cmake from pip (modern version) instead of system cmake 3.10
CMAKE_BIN="$VENV/bin/cmake"
if [ ! -x "$CMAKE_BIN" ]; then
    CMAKE_BIN="cmake"
fi
echo "CMake  : $($CMAKE_BIN --version | head -1)"

$CMAKE_BIN "$BINDINGS_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE="$PYTHON" \
    -Dpybind11_DIR="$PYBIND11_CMAKE"

make -j3

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
sudo make install

# ---------------------------------------------------------------------------
# Register libORB_SLAM3.so so the OS can find it at runtime
# ---------------------------------------------------------------------------
echo "/opt/ORB_SLAM3/lib" | sudo tee /etc/ld.so.conf.d/orbslam3.conf > /dev/null
sudo ldconfig

# ---------------------------------------------------------------------------
# Smoke test
# ---------------------------------------------------------------------------
echo ""
echo "Testing import..."
$PYTHON -c "import orbslam3; print('OK -- available:', [x for x in dir(orbslam3) if not x.startswith('_')])"
echo ""
echo "Done. Restart src.main to use real SLAM."
