#pragma once

#include <memory>
#include <opencv2/core.hpp>
#include <rclcpp/time.hpp>

namespace passive_stereo_capture
{

/// Holds a synchronized, rectified, CLAHE-enhanced stereo pair.
/// Shared across all worker threads via shared_ptr — zero-copy hand-off.
struct StereoFrame
{
    cv::Mat  left;              ///< RGB8, rectified + CLAHE
    cv::Mat  right;             ///< RGB8, rectified + CLAHE
    double   timestamp_sec{0.0};
    rclcpp::Time stamp;
    uint64_t frame_id{0};      ///< From Spinnaker chunk data (FrameID)
};

using StereoFramePtr = std::shared_ptr<StereoFrame>;

}  // namespace passive_stereo_capture
