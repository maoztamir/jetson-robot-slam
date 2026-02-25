cmake_minimum_required(VERSION 3.12)
project(orbslam3_bindings CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# ---------------------------------------------------------------------------
# Python -- use PYTHON_EXECUTABLE passed in from the build script so we
# always target the active venv (Ubuntu 18.04 CMake lacks find_package(Python3))
# ---------------------------------------------------------------------------
if(NOT DEFINED PYTHON_EXECUTABLE)
    set(PYTHON_EXECUTABLE "python3")
endif()
message(STATUS "Python: ${PYTHON_EXECUTABLE}")

# pybind11 from pip (installed in venv)
execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c
        "import pybind11, os; print(os.path.join(os.path.dirname(pybind11.__file__), 'share', 'cmake', 'pybind11'))"
    OUTPUT_VARIABLE pybind11_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
find_package(pybind11 REQUIRED)

# Python include/lib via the venv interpreter
execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c
        "import sysconfig; print(sysconfig.get_path('include'))"
    OUTPUT_VARIABLE PYTHON_INCLUDE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c
        "import sysconfig; print(sysconfig.get_config_var('LDLIBRARY'))"
    OUTPUT_VARIABLE _pylib
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c
        "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))"
    OUTPUT_VARIABLE _pylibdir
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
set(PYTHON_LIBRARY "${_pylibdir}/${_pylib}")
message(STATUS "Python include: ${PYTHON_INCLUDE_DIR}")
message(STATUS "Python library: ${PYTHON_LIBRARY}")

include_directories(${PYTHON_INCLUDE_DIR})

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
    COMMAND "${PYTHON_EXECUTABLE}" -c "import site; print(site.getsitepackages()[0])"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS "Installing to: ${PYTHON_SITE_PACKAGES}")

install(TARGETS orbslam3
    LIBRARY DESTINATION "${PYTHON_SITE_PACKAGES}"
)
