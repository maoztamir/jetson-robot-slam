cmake_minimum_required(VERSION 3.10)
project(orbslam3_bindings CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# ---------------------------------------------------------------------------
# Python -- use PYTHON_EXECUTABLE passed from the build script
# ---------------------------------------------------------------------------
if(NOT DEFINED PYTHON_EXECUTABLE)
    set(PYTHON_EXECUTABLE "python3")
endif()
message(STATUS "Python: ${PYTHON_EXECUTABLE}")

# Python headers (from the venv)
execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c "import sysconfig; print(sysconfig.get_path('include'))"
    OUTPUT_VARIABLE PYTHON_INCLUDE_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
)

# pybind11 headers (get_include() exists in all pybind11 versions)
execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c "import pybind11; print(pybind11.get_include())"
    OUTPUT_VARIABLE PYBIND11_INCLUDE_DIR OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Python extension suffix  (.cpython-38-aarch64-linux-gnu.so)
execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))"
    OUTPUT_VARIABLE PYTHON_EXT_SUFFIX OUTPUT_STRIP_TRAILING_WHITESPACE
)

message(STATUS "Python include : ${PYTHON_INCLUDE_DIR}")
message(STATUS "pybind11 include: ${PYBIND11_INCLUDE_DIR}")
message(STATUS "Extension suffix: ${PYTHON_EXT_SUFFIX}")

# ---------------------------------------------------------------------------
# Other dependencies
# ---------------------------------------------------------------------------
# The bindings MUST link against the same OpenCV version that libORB_SLAM3.so
# was compiled against — otherwise both versions load simultaneously and crash.
# Check with: ldd /opt/ORB_SLAM3/lib/libORB_SLAM3.so | grep opencv
#
# Pass the correct path via -DOpenCV_DIR=... on the cmake command line:
#   OpenCV 4.1 (tele3): -DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4
#   OpenCV 4.5.5 (tele2): -DOpenCV_DIR=/usr/local/lib/cmake/opencv4
#
# Default falls back to 4.1 (system) if not specified.
if(NOT DEFINED OpenCV_DIR)
    set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4")
endif()
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(ORB_SLAM3_DIR /opt/ORB_SLAM3)

include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/stub_pangolin"  # intercepts <pangolin/pangolin.h> before /usr/local/include
    ${PYTHON_INCLUDE_DIR}
    ${PYBIND11_INCLUDE_DIR}
    ${ORB_SLAM3_DIR}                          # needed for "Thirdparty/..." relative includes
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/include/CameraModels
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/DBoW2
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# ---------------------------------------------------------------------------
# Build the extension module (no find_package(pybind11) needed)
# ---------------------------------------------------------------------------
add_library(orbslam3 MODULE orbslam3_bindings.cpp)

set_target_properties(orbslam3 PROPERTIES
    PREFIX ""
    SUFFIX "${PYTHON_EXT_SUFFIX}"
    INSTALL_RPATH "${ORB_SLAM3_DIR}/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# pybind11 requires symbols to be hidden by default
target_compile_options(orbslam3 PRIVATE -fvisibility=hidden)

target_link_libraries(orbslam3 PRIVATE
    ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
    ${OpenCV_LIBS}
)

# ---------------------------------------------------------------------------
# Install to venv site-packages
# ---------------------------------------------------------------------------
execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c "import site; print(site.getsitepackages()[0])"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS "Site-packages  : ${PYTHON_SITE_PACKAGES}")

install(TARGETS orbslam3
    LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}"
)
