#pragma once
// Stub Viewer.h — replaces the real ORB_SLAM3 Viewer.h during binding
// compilation to prevent Pangolin GL headers from being pulled in.
// We never instantiate a Viewer (use_viewer=false), so no implementation needed.
namespace ORB_SLAM3 {
class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class System;
class Viewer {
public:
    Viewer(System*, FrameDrawer*, MapDrawer*, Tracking*, const std::string&,
           const std::string&) {}
    void Run() {}
    void RequestFinish() {}
    void RequestStop() {}
    bool isFinished() { return true; }
    bool isStopped() { return true; }
    bool Stop() { return true; }
    void Release() {}
    void SetTrackingPause() {}
};
} // namespace ORB_SLAM3
