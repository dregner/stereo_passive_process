#pragma once

#include <string>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

namespace passive_stereo_capture
{

/// Loads stereo calibration from an OpenCV FileStorage YAML and pre-computes
/// rectification maps for both cameras using cv::stereoRectify.
///
/// The YAML file must contain:
///   image_width, image_height — native (unrectified) image dimensions
///   K1, D1 — left camera 3x3 intrinsic matrix and distortion vector
///   K2, D2 — right camera 3x3 intrinsic matrix and distortion vector
///   R       — rotation of right w.r.t. left (3x3)
///   T       — translation of right w.r.t. left (3x1, metres)
class StereoRectifier
{
public:
    StereoRectifier() = default;

    /// Load calibration and compute maps.
    /// @throws std::runtime_error if the file is missing or malformed.
    void load(const std::string & calib_yaml_path);

    /// Rectify a raw stereo pair (debayered RGB8).
    /// Returns the rectified images at the same resolution as the input.
    void rectify(const cv::Mat & left_raw,
                 const cv::Mat & right_raw,
                 cv::Mat & left_rect,
                 cv::Mat & right_rect) const;

    bool isLoaded() const { return loaded_; }

    // Accessors for downstream consumers (Retinify, SLAM config)
    int width()      const { return rect_size_.width;  }
    int height()     const { return rect_size_.height; }
    double fx()      const { return P1_.at<double>(0, 0); }
    double fy()      const { return P1_.at<double>(1, 1); }
    double cx()      const { return P1_.at<double>(0, 2); }
    double cy()      const { return P1_.at<double>(1, 2); }
    double baseline() const { return baseline_m_; }  ///< metres
    const cv::Mat & Q() const { return Q_; }          ///< disparity-to-depth mapping
    const cv::Mat & P1() const { return P1_; }
    const cv::Mat & P2() const { return P2_; }

private:
    cv::Mat map1_left_,  map2_left_;    ///< remap maps for left camera
    cv::Mat map1_right_, map2_right_;   ///< remap maps for right camera
    cv::Mat P1_, P2_, Q_;
    cv::Size rect_size_;
    double baseline_m_{0.0};
    bool loaded_{false};
};

}  // namespace passive_stereo_capture
