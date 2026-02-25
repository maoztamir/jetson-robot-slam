cmake_minimum_required(VERSION 3.10)
project(orbslam3_bindings CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# ---------------------------------------------------------------------------
# Python (use the active venv's interpreter)
# ---------------------------------------------------------------------------
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
message(STATUS "Python: ${Python3_EXECUTABLE}")

# pybind11 from pip (installed in venv)
execute_process(
    COMMAND "${Python3_EXECUTABLE}" -m pybind11 --cmakedir
    OUTPUT_VARIABLE pybind11_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
find_package(pybind11 REQUIRED)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(ORB_SLAM3_DIR /opt/ORB_SLAM3)

include_directories(
    ${ORB_SLAM3_DIR}/include
    ${ORB_SLAM3_DIR}/include/CameraModels
    ${ORB_SLAM3_DIR}/Thirdparty/Sophus
    ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# ---------------------------------------------------------------------------
# Build the extension module
# ---------------------------------------------------------------------------
pybind11_add_module(orbslam3 orbslam3_bindings.cpp)

target_link_libraries(orbslam3 PRIVATE
    ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
    ${OpenCV_LIBS}
)

# Embed RPATH so orbslam3.so finds libORB_SLAM3.so at runtime without
# needing LD_LIBRARY_PATH to be set.
set_target_properties(orbslam3 PROPERTIES
    INSTALL_RPATH "${ORB_SLAM3_DIR}/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# ---------------------------------------------------------------------------
# Install to venv site-packages
# ---------------------------------------------------------------------------
execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import site; print(site.getsitepackages()[0])"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS "Installing to: ${PYTHON_SITE_PACKAGES}")

install(TARGETS orbslam3
    LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}"
)
