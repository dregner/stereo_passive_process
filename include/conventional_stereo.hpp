#pragma once

#include <limits>
#include <stdexcept>
#include <string>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/stereo.hpp>

namespace passive_stereo_capture {

// CPU matcher for rectified RGB pairs. Disparity is returned in pixels.
class ConventionalStereo {
public:
    struct Config {
        int min_disparity{0};
        int num_disparities{128};
        int block_size{9};
        int uniqueness_ratio{10};
        int speckle_window_size{100};
        int speckle_range{2};
        int disp12_max_diff{1};
        int pre_filter_cap{31};
        int texture_threshold{10}; // BM only
    };

    ConventionalStereo(const std::string & backend, const Config & cfg) : cfg_(cfg) {
        if (backend != "stereobm" && backend != "stereosgbm" && backend != "stereobinary")
            throw std::invalid_argument("Unknown conventional stereo backend");
        if (cfg.num_disparities <= 0 || cfg.num_disparities % 16 != 0 ||
            cfg.min_disparity < 0 || cfg.min_disparity + static_cast<long long>(cfg.num_disparities) > 2047 ||
            cfg.block_size < (backend == "stereobm" ? 5 : 1) || cfg.block_size > 255 ||
            cfg.block_size % 2 == 0 || cfg.uniqueness_ratio < 0 ||
            cfg.speckle_window_size < 0 || cfg.speckle_range < 0 ||
            cfg.pre_filter_cap < 1 || cfg.pre_filter_cap > 63 || cfg.texture_threshold < 0)
            throw std::invalid_argument("Invalid stereo matcher parameters: disparities must be a positive multiple of 16, block size odd (BM >=5), and filters nonnegative");
        if (backend == "stereobinary") {
            binary_matcher_ = cv::stereo::StereoBinarySGBM::create(
                cfg.min_disparity, cfg.num_disparities, cfg.block_size,
                100, 1000, cfg.disp12_max_diff, cfg.pre_filter_cap,
                cfg.uniqueness_ratio, cfg.speckle_window_size, cfg.speckle_range,
                cv::stereo::StereoBinarySGBM::MODE_SGBM);
            return;
        }
        if (backend == "stereobm") {
            auto bm = cv::StereoBM::create(cfg.num_disparities, cfg.block_size);
            bm->setUniquenessRatio(cfg.uniqueness_ratio);
            bm->setTextureThreshold(cfg.texture_threshold);
            bm->setPreFilterCap(cfg.pre_filter_cap);
            matcher_ = bm;
        } else {
            matcher_ = cv::StereoSGBM::create(cfg.min_disparity, cfg.num_disparities,
                cfg.block_size, 8 * cfg.block_size * cfg.block_size,
                32 * cfg.block_size * cfg.block_size, cfg.disp12_max_diff,
                cfg.pre_filter_cap, cfg.uniqueness_ratio, cfg.speckle_window_size,
                cfg.speckle_range, cv::StereoSGBM::MODE_SGBM_3WAY);
        }
        matcher_->setMinDisparity(cfg.min_disparity);
        matcher_->setSpeckleWindowSize(cfg.speckle_window_size);
        matcher_->setSpeckleRange(cfg.speckle_range);
        matcher_->setDisp12MaxDiff(cfg.disp12_max_diff);
    }

    void compute(const cv::Mat & left_rgb, const cv::Mat & right_rgb, cv::Mat & disparity) {
        if (left_rgb.type() != CV_8UC3 || right_rgb.type() != CV_8UC3 ||
            left_rgb.size() != right_rgb.size() ||
            left_rgb.cols <= cfg_.min_disparity + cfg_.num_disparities + cfg_.block_size ||
            left_rgb.rows <= cfg_.block_size)
            throw std::invalid_argument("Stereo pair must be equally sized RGB8 images larger than the disparity search and block size");
        cv::cvtColor(left_rgb, left_gray_, cv::COLOR_RGB2GRAY);
        cv::cvtColor(right_rgb, right_gray_, cv::COLOR_RGB2GRAY);
        if (binary_matcher_) binary_matcher_->compute(left_gray_, right_gray_, fixed_);
        else matcher_->compute(left_gray_, right_gray_, fixed_);
        fixed_.convertTo(disparity, CV_32F, 1.0 / 16.0);
        if (binary_matcher_) {
            // Binary matching can assign arbitrary positive disparity on constant patches.
            cv::Mat gray_float, mean, mean_square;
            left_gray_.convertTo(gray_float, CV_32F);
            const cv::Size window(cfg_.block_size, cfg_.block_size);
            cv::blur(gray_float, mean, window);
            cv::blur(gray_float.mul(gray_float), mean_square, window);
            disparity.setTo(std::numeric_limits<float>::quiet_NaN(),
                mean_square - mean.mul(mean) < 1.0f);
        }
        // Invalid matches must never become plausible XYZ, even without confidence filtering.
        disparity.setTo(std::numeric_limits<float>::quiet_NaN(),
            (fixed_ <= (cfg_.min_disparity - 1) * 16) | (fixed_ <= 0));
    }

    static cv::Mat scaledQ(const cv::Mat & native_q, cv::Size native, cv::Size output) {
        const double sx = static_cast<double>(output.width) / native.width;
        const double sy = static_cast<double>(output.height) / native.height;
        // Map resized pixel centres and horizontal disparity back to calibration coordinates.
        cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
        transform.at<double>(0, 0) = 1.0 / sx;
        transform.at<double>(1, 1) = 1.0 / sy;
        transform.at<double>(2, 2) = 1.0 / sx;
        transform.at<double>(0, 3) = 0.5 / sx - 0.5;
        transform.at<double>(1, 3) = 0.5 / sy - 0.5;
        return native_q * transform;
    }

private:
    Config cfg_;
    cv::Ptr<cv::StereoMatcher> matcher_;
    cv::Ptr<cv::stereo::StereoBinarySGBM> binary_matcher_;
    cv::Mat left_gray_, right_gray_, fixed_;
};
}
