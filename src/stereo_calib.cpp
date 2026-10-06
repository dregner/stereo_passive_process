#include "stereo_calib.hpp"

#include <stdexcept>
#include <opencv2/imgproc.hpp>

namespace passive_stereo_capture
{
retinify::DistortionCoefficients StereoCalib::toRetinifyDistortion(const cv::Mat & dist_mat){
    
    retinify::DistortionCoefficients dist{};
    if (dist_mat.empty()) {
        return dist;
    }
    // Ensure we can access as double
    cv::Mat d64;
    dist_mat.convertTo(d64, CV_64F);
    const double * d = d64.ptr<double>();
    const int n = static_cast<int>(d64.total());
    // OpenCV order is: k1, k2, p1, p2, k3, k4, k5, k6
    if (n >= 1) dist.k1 = d[0];
    if (n >= 2) dist.k2 = d[1];
    if (n >= 3) dist.p1 = d[2];
    if (n >= 4) dist.p2 = d[3];
    if (n >= 5) dist.k3 = d[4];
    if (n >= 6) dist.k4 = d[5];
    if (n >= 7) dist.k5 = d[6];
    if (n >= 8) dist.k6 = d[7];
    return dist;
}
void StereoCalib::load(const std::string & calib_yaml_path)
{
    cv::FileStorage fs(calib_yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error(
            "StereoCalib: cannot open calibration file: " + calib_yaml_path);
    }

    int w{0}, h{0};
    fs["image_width"]  >> w;
    fs["image_height"] >> h;
    if (w <= 0 || h <= 0) {
        throw std::runtime_error(
            "StereoCalib: missing image_width / image_height in " + calib_yaml_path);
    }
    img_size_ = cv::Size(w, h);

    fs["K1"] >> K1_;
    fs["D1"] >> D1_;
    fs["K2"] >> K2_;
    fs["D2"] >> D2_;
    fs["R"]  >> R_;
    fs["T"]  >> T_;
    fs.release();

    for (const auto* m : {&K1_, &D1_, &K2_, &D2_, &R_, &T_}) {
        if (m->empty()) {
            throw std::runtime_error(
                "StereoCalib: one or more required matrices (K1/D1/K2/D2/R/T) "
                "is empty in " + calib_yaml_path);
        }
    }

    // Ensure double precision
    if (K1_.type() != CV_64F) K1_.convertTo(K1_, CV_64F);
    if (K2_.type() != CV_64F) K2_.convertTo(K2_, CV_64F);
    if (D1_.type() != CV_64F) D1_.convertTo(D1_, CV_64F);
    if (D2_.type() != CV_64F) D2_.convertTo(D2_, CV_64F);
    if (R_.type()  != CV_64F) R_.convertTo(R_, CV_64F);
    if (T_.type()  != CV_64F) T_.convertTo(T_, CV_64F);

    cv::Mat R1, R2;
    cv::stereoRectify(K1_, D1_, K2_, D2_, img_size_, R_, T_, R1, R2, P1_, P2_, Q_,
        cv::CALIB_ZERO_DISPARITY, /*alpha=*/0.0, img_size_);

    // Pre-compute undistort + rectify maps
    cv::initUndistortRectifyMap(K1_, D1_, R1, P1_, img_size_, CV_16SC2,
                                map1_left_, map2_left_);
    cv::initUndistortRectifyMap(K2_, D2_, R2, P2_, img_size_, CV_16SC2,
                                map1_right_, map2_right_);
    baseline_m_ = std::abs(P2_.at<double>(0,3)/P2_.at<double>(0,0));
    loaded_ = true;
}

void StereoCalib::rectify( const cv::Mat & left_raw, const cv::Mat & right_raw, cv::Mat & left_rect, cv::Mat & right_rect) const
{
    cv::remap(left_raw,  left_rect,  map1_left_,  map2_left_,  cv::INTER_LINEAR);
    cv::remap(right_raw, right_rect, map1_right_, map2_right_, cv::INTER_LINEAR);
}

}  // namespace passive_stereo_capture
