#pragma once

#include <memory>
#include <cstdint>
#include <chrono>
#include <opencv2/core.hpp>
#include <rclcpp/time.hpp>

namespace passive_stereo_capture
{

struct StereoFrame
{
    cv::Mat  left_raw;  ///< CV_8UC1 BayerRG8, unrectified
    cv::Mat  right_raw; ///< CV_8UC1 BayerRG8, unrectified
    cv::Mat left_rgb;
    cv::Mat right_rgb;
    
    std::chrono::steady_clock::time_point received_at;
    uint64_t     timestamp_ns{0};    ///< Spinnaker chunk timestamp in nanoseconds
    double       timestamp_sec{0.0}; ///< Same, as seconds (for TrackStereo)
    rclcpp::Time stamp;
    uint64_t     frame_id{0};        ///< From Spinnaker chunk data (FrameID)
};

using StereoFramePtr = std::shared_ptr<StereoFrame>;

}  // namespace passive_stereo_capture
