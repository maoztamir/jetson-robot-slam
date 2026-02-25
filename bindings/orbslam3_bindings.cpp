/**
 * ORB_SLAM3 pybind11 bindings
 *
 * Exposes exactly the API used by src/slam/orb_slam3_wrapper.py:
 *
 *   orbslam3.Sensor.STEREO
 *   orbslam3.Sensor.STEREO_INERTIAL
 *   orbslam3.System(vocab, settings, sensor, use_viewer=False)
 *   system.process_image_stereo(left, right, timestamp)          -> np.ndarray (4x4)
 *   system.process_image_stereo_inertial(left, right, ts, imu)  -> np.ndarray (4x4)
 *   system.get_tracking_state()                                  -> int
 *   system.get_tracked_mappoints()                               -> list of (x,y,z)
 *   system.shutdown()
 *
 * IMU array columns: [timestamp, ax, ay, az, gx, gy, gz]  (float64, shape Nx7)
 * Pose return: Tcw as float64 4x4 numpy array (camera <- world).
 *              The Python wrapper inverts this to get Twc.
 */

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <opencv2/core/core.hpp>
#include <Eigen/Core>

#include <System.h>
#include <ImuTypes.h>
#include <MapPoint.h>

namespace py = pybind11;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static cv::Mat numpy_to_mat(py::array_t<uint8_t> arr)
{
    py::buffer_info buf = arr.request();
    return cv::Mat(
        static_cast<int>(buf.shape[0]),
        static_cast<int>(buf.shape[1]),
        CV_8UC3,
        buf.ptr
    ).clone();
}

static py::array_t<double> se3f_to_numpy(const Sophus::SE3f& pose)
{
    Eigen::Matrix4f m = pose.matrix();
    py::array_t<double> out({4, 4});
    auto w = out.mutable_unchecked<2>();
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            w(r, c) = static_cast<double>(m(r, c));
    return out;
}

// ---------------------------------------------------------------------------
// Module
// ---------------------------------------------------------------------------

PYBIND11_MODULE(orbslam3, m)
{
    m.doc() = "ORB_SLAM3 pybind11 bindings";

    // Sensor enum
    py::enum_<ORB_SLAM3::System::eSensor>(m, "Sensor")
        .value("STEREO",          ORB_SLAM3::System::STEREO)
        .value("STEREO_INERTIAL", ORB_SLAM3::System::IMU_STEREO)
        .export_values();

    // System class
    py::class_<ORB_SLAM3::System>(m, "System")
        .def(py::init([](const std::string& vocab,
                         const std::string& settings,
                         ORB_SLAM3::System::eSensor sensor,
                         bool viewer) {
            return new ORB_SLAM3::System(vocab, settings, sensor, viewer);
        }),
        py::arg("vocab"),
        py::arg("settings"),
        py::arg("sensor"),
        py::arg("use_viewer") = false)

        // Stereo (no IMU)
        .def("process_image_stereo",
            [](ORB_SLAM3::System& self,
               py::array_t<uint8_t> left,
               py::array_t<uint8_t> right,
               double ts) {
                cv::Mat l = numpy_to_mat(left);
                cv::Mat r = numpy_to_mat(right);
                Sophus::SE3f Tcw = self.TrackStereo(l, r, ts);
                return se3f_to_numpy(Tcw);
            })

        // Stereo + IMU  (Nx7 array: [t, ax, ay, az, gx, gy, gz])
        .def("process_image_stereo_inertial",
            [](ORB_SLAM3::System& self,
               py::array_t<uint8_t> left,
               py::array_t<uint8_t> right,
               double ts,
               py::array_t<double> imu_arr) {
                cv::Mat l = numpy_to_mat(left);
                cv::Mat r = numpy_to_mat(right);

                std::vector<ORB_SLAM3::IMU::Point> imu;
                auto a = imu_arr.unchecked<2>();
                for (ssize_t i = 0; i < a.shape(0); i++) {
                    imu.emplace_back(
                        float(a(i, 1)), float(a(i, 2)), float(a(i, 3)),  // accel x,y,z
                        float(a(i, 4)), float(a(i, 5)), float(a(i, 6)),  // gyro  x,y,z
                        a(i, 0)                                           // timestamp
                    );
                }

                Sophus::SE3f Tcw = self.TrackStereo(l, r, ts, imu);
                return se3f_to_numpy(Tcw);
            })

        .def("get_tracking_state", &ORB_SLAM3::System::GetTrackingState)

        .def("get_tracked_mappoints",
            [](ORB_SLAM3::System& self) {
                py::list out;
                for (auto* mp : self.GetTrackedMapPoints()) {
                    if (mp && !mp->isBad()) {
                        Eigen::Vector3f pos = mp->GetWorldPos();
                        out.append(py::make_tuple(pos.x(), pos.y(), pos.z()));
                    }
                }
                return out;
            })

        .def("shutdown", &ORB_SLAM3::System::Shutdown);
}
