#pragma once

#include <memory>
#include <cstdint>
#include <opencv2/core.hpp>
#include <rclcpp/time.hpp>

namespace passive_stereo_capture
{

/// Holds a synchronized, debayered stereo pair for fan-out to all workers.
/// Shared across all worker threads via shared_ptr — zero-copy hand-off.
///
/// Pipeline:
///   left_gray  — Bayer→GRAY (direct) → rectify  — for SLAM (resize+CLAHE inside SlamWorker)
///   left_rgb   — Bayer→RGB → rectify → CLAHE     — for Retinify and Preview (shared)
struct StereoFrame
{
    // SLAM path: rectified grayscale (full-res).
    // BayerRG2GRAY is a single-step direct conversion, much cheaper than full debayer.
    // SlamWorker resizes and applies grayscale CLAHE internally.
    cv::Mat  left_gray;    ///< CV_8UC1, rectified (no CLAHE)
    cv::Mat  right_gray;   ///< CV_8UC1, rectified (no CLAHE)

    // Disparity path: rectified RGB8 with CLAHE applied in CIE Lab L-channel.
    cv::Mat  left_rgb;     ///< CV_8UC3 RGB8, rectified + CLAHE
    cv::Mat  right_rgb;    ///< CV_8UC3 RGB8, rectified + CLAHE

    // Preview path: unrectified (raw perspective) BGR8.
    // Preview does not rectify images — publishes BGR color format standard for ROS 2.
    cv::Mat  left_raw_bgr;  ///< CV_8UC3 BGR8, raw perspective (unrectified)
    cv::Mat  right_raw_bgr; ///< CV_8UC3 BGR8, raw perspective (unrectified)

    uint64_t     timestamp_ns{0};    ///< Spinnaker chunk timestamp in nanoseconds
    double       timestamp_sec{0.0}; ///< Same, as seconds (for TrackStereo)
    rclcpp::Time stamp;
    uint64_t     frame_id{0};        ///< From Spinnaker chunk data (FrameID)
};

using StereoFramePtr = std::shared_ptr<StereoFrame>;

}  // namespace passive_stereo_capture
