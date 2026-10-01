#include "stereo_rectifier.hpp"

#include <stdexcept>
#include <opencv2/imgproc.hpp>

namespace passive_stereo_capture
{

void StereoRectifier::load(const std::string & calib_yaml_path)
{
    cv::FileStorage fs(calib_yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error(
            "StereoRectifier: cannot open calibration file: " + calib_yaml_path);
    }

    int w{0}, h{0};
    fs["image_width"]  >> w;
    fs["image_height"] >> h;
    if (w <= 0 || h <= 0) {
        throw std::runtime_error(
            "StereoRectifier: missing image_width / image_height in " + calib_yaml_path);
    }
    cv::Size img_size(w, h);

    cv::Mat K1, D1, K2, D2, R, T;
    fs["K1"] >> K1;
    fs["D1"] >> D1;
    fs["K2"] >> K2;
    fs["D2"] >> D2;
    fs["R"]  >> R;
    fs["T"]  >> T;
    fs.release();

    for (const auto* m : {&K1, &D1, &K2, &D2, &R, &T}) {
        if (m->empty()) {
            throw std::runtime_error(
                "StereoRectifier: one or more required matrices (K1/D1/K2/D2/R/T) "
                "is empty in " + calib_yaml_path);
        }
    }

    // Ensure double precision
    if (K1.type() != CV_64F) K1.convertTo(K1, CV_64F);
    if (K2.type() != CV_64F) K2.convertTo(K2, CV_64F);
    if (D1.type() != CV_64F) D1.convertTo(D1, CV_64F);
    if (D2.type() != CV_64F) D2.convertTo(D2, CV_64F);
    if (R.type()  != CV_64F) R.convertTo(R, CV_64F);
    if (T.type()  != CV_64F) T.convertTo(T, CV_64F);

    // Compute rectification transforms
    cv::Mat R1, R2;
    cv::stereoRectify(
        K1, D1, K2, D2, img_size, R, T,
        R1, R2, P1_, P2_, Q_,
        cv::CALIB_ZERO_DISPARITY,
        /*alpha=*/0.0,  // crop to valid pixels only
        img_size);

    rect_size_ = img_size;

    // Pre-compute undistort + rectify maps
    cv::initUndistortRectifyMap(K1, D1, R1, P1_, rect_size_, CV_16SC2,
                                map1_left_, map2_left_);
    cv::initUndistortRectifyMap(K2, D2, R2, P2_, rect_size_, CV_16SC2,
                                map1_right_, map2_right_);

    // Baseline: from P2 = [fx 0 cx -fx*B; ...]
    baseline_m_ = std::abs(P2_.at<double>(0, 3) / P2_.at<double>(0, 0));

    loaded_ = true;
}

void StereoRectifier::rectify(
    const cv::Mat & left_raw,
    const cv::Mat & right_raw,
    cv::Mat & left_rect,
    cv::Mat & right_rect) const
{
    cv::remap(left_raw,  left_rect,  map1_left_,  map2_left_,  cv::INTER_LINEAR);
    cv::remap(right_raw, right_rect, map1_right_, map2_right_, cv::INTER_LINEAR);
}

}  // namespace passive_stereo_capture
