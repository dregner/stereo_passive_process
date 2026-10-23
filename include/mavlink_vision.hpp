#pragma once

#include <Eigen/Geometry>
#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace passive_stereo_capture
{
// Vehicle base pose: local forward/left/up map, forward/left/up body.
struct VisionSample {
    uint64_t usec{0};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    uint64_t epoch{0};
};

inline Eigen::Matrix3d fluToFrd()
{
    return Eigen::Vector3d(1, -1, -1).asDiagonal();
}

inline Eigen::Matrix3d visionRotation(const VisionSample & sample, double yaw)
{
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        fluToFrd() * sample.orientation.toRotationMatrix() * fluToFrd();
}

inline Eigen::Vector3d visionPosition(const VisionSample & sample, double yaw)
{
    Eigen::Vector3d p(sample.position.x(), sample.position.y(), 0.0);
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * (fluToFrd() * p);
}

inline std::array<float, 3> visionRpy(const Eigen::Matrix3d & r)
{
    return {static_cast<float>(std::atan2(r(2,1), r(2,2))),
            static_cast<float>(std::asin(std::clamp(-r(2,0), -1.0, 1.0))),
            static_cast<float>(std::atan2(r(1,0), r(0,0)))};
}

inline Eigen::Vector3d visionRotationVector(const Eigen::Matrix3d & r)
{
    Eigen::AngleAxisd aa(r);
    return aa.axis() * aa.angle();
}
}  // namespace passive_stereo_capture
