#pragma once

#include <limits>
#include <stdexcept>
#include <string>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#ifndef PASSIVE_DISABLE_BINARY_STEREO
#include <opencv2/stereo.hpp>
#endif
#ifdef PASSIVE_HAVE_OPENCV_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudastereo.hpp>
#endif

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
        int cuda_p1{10};
        int cuda_p2{120};
    };

    ConventionalStereo(const std::string & backend, const Config & cfg) : cfg_(cfg) {
        if (backend != "stereobm" && backend != "stereosgbm" && backend != "stereobinary" && backend != "cuda_sgm")
            throw std::invalid_argument("Unknown conventional stereo backend");
        if (cfg.num_disparities <= 0 || cfg.num_disparities % 16 != 0 ||
            cfg.min_disparity < 0 || cfg.min_disparity + static_cast<long long>(cfg.num_disparities) > 2047 ||
            cfg.block_size < (backend == "stereobm" ? 5 : 1) || cfg.block_size > 255 ||
            cfg.block_size % 2 == 0 || cfg.uniqueness_ratio < 0 ||
            cfg.speckle_window_size < 0 || cfg.speckle_range < 0 ||
            cfg.pre_filter_cap < 1 || cfg.pre_filter_cap > 63 || cfg.texture_threshold < 0)
            throw std::invalid_argument("Invalid stereo matcher parameters: disparities must be a positive multiple of 16, block size odd (BM >=5), and filters nonnegative");
        if (backend == "cuda_sgm") {
            if ((cfg.num_disparities != 64 && cfg.num_disparities != 128 && cfg.num_disparities != 256) ||
                cfg.cuda_p1 < 0 || cfg.cuda_p2 <= cfg.cuda_p1)
                throw std::invalid_argument("CUDA SGM needs 64, 128 or 256 disparities and 0 <= P1 < P2");
#ifdef PASSIVE_HAVE_OPENCV_CUDA
            ensureCudaStream();
            cuda_matcher_ = cv::cuda::createStereoSGM(cfg.min_disparity, cfg.num_disparities,
                cfg.cuda_p1, cfg.cuda_p2, cfg.uniqueness_ratio, cv::cuda::StereoSGM::MODE_HH4);
            return;
#else
            throw std::invalid_argument("cuda_sgm requires OpenCV CUDA stereo/imgproc modules");
#endif
        }
        if (backend == "stereobinary") {
#ifndef PASSIVE_DISABLE_BINARY_STEREO
            binary_matcher_ = cv::stereo::StereoBinarySGBM::create(
                cfg.min_disparity, cfg.num_disparities, cfg.block_size,
                100, 1000, cfg.disp12_max_diff, cfg.pre_filter_cap,
                cfg.uniqueness_ratio, cfg.speckle_window_size, cfg.speckle_range,
                cv::stereo::StereoBinarySGBM::MODE_SGBM);
#else
            throw std::invalid_argument("stereobinary requires the OpenCV contrib stereo module");
#endif
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
#ifdef PASSIVE_HAVE_OPENCV_CUDA
        if (cuda_matcher_) {
            cuda_left_.upload(left_rgb, *cuda_stream_);
            cuda_right_.upload(right_rgb, *cuda_stream_);
            cv::cuda::cvtColor(cuda_left_, cuda_left_gray_, cv::COLOR_RGB2GRAY, 0, *cuda_stream_);
            cv::cuda::cvtColor(cuda_right_, cuda_right_gray_, cv::COLOR_RGB2GRAY, 0, *cuda_stream_);
            cuda_matcher_->compute(cuda_left_gray_, cuda_right_gray_, cuda_fixed_, *cuda_stream_);
            // SGM produces CV_16S with four fractional bits. Reprojection expects pixels.
            cuda_fixed_.convertTo(cuda_pixels_, CV_32F, 1.0 / 16.0, 0.0, *cuda_stream_);
            cuda_pixels_.download(disparity, *cuda_stream_);
            cuda_stream_->waitForCompletion();
            disparity.setTo(std::numeric_limits<float>::quiet_NaN(),
                (disparity <= cfg_.min_disparity - 1) | (disparity <= 0));
            return;
        }
#endif
        cv::cvtColor(left_rgb, left_gray_, cv::COLOR_RGB2GRAY);
        cv::cvtColor(right_rgb, right_gray_, cv::COLOR_RGB2GRAY);
#ifndef PASSIVE_DISABLE_BINARY_STEREO
        if (binary_matcher_) binary_matcher_->compute(left_gray_, right_gray_, fixed_);
        else
#endif
        matcher_->compute(left_gray_, right_gray_, fixed_);
        fixed_.convertTo(disparity, CV_32F, 1.0 / 16.0);
#ifndef PASSIVE_DISABLE_BINARY_STEREO
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
#endif
        // Invalid matches must never become plausible XYZ, even without confidence filtering.
        disparity.setTo(std::numeric_limits<float>::quiet_NaN(),
            (fixed_ <= (cfg_.min_disparity - 1) * 16) | (fixed_ <= 0));
    }

    bool isCuda() const {
#ifdef PASSIVE_HAVE_OPENCV_CUDA
        return !cuda_matcher_.empty();
#else
        return false;
#endif
    }

    // CUDA matching retains its float pixel disparity on-device. CPU matching can
    // optionally upload disparity for GPU reprojection; cloud packing remains CPU.
    void reproject(const cv::Mat & disparity, cv::Mat & xyz, const cv::Mat & q, bool use_cuda) {
        if (!use_cuda) {
            cv::reprojectImageTo3D(disparity, xyz, q, false, CV_32F);
            return;
        }
#ifdef PASSIVE_HAVE_OPENCV_CUDA
        ensureCudaStream();
        if (!cuda_matcher_) cuda_pixels_.upload(disparity, *cuda_stream_);
        cv::Mat float_q;
        q.convertTo(float_q, CV_32F);
        cv::cuda::reprojectImageTo3D(cuda_pixels_, cuda_xyz_, float_q, 3, *cuda_stream_);
        cuda_xyz_.download(xyz, *cuda_stream_);
        cuda_stream_->waitForCompletion();
        xyz.setTo(cv::Scalar::all(std::numeric_limits<float>::quiet_NaN()), disparity != disparity);
#else
        throw std::invalid_argument("GPU reprojection requires the OpenCV CUDA stereo module");
#endif
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
#ifndef PASSIVE_DISABLE_BINARY_STEREO
    cv::Ptr<cv::stereo::StereoBinarySGBM> binary_matcher_;
#endif
    cv::Mat left_gray_, right_gray_, fixed_;
#ifdef PASSIVE_HAVE_OPENCV_CUDA
    void ensureCudaStream() {
        if (cuda_stream_) return;
        if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
            throw std::invalid_argument("OpenCV CUDA processing requested but no CUDA device is available");
        cuda_stream_ = std::make_unique<cv::cuda::Stream>();
    }
    std::unique_ptr<cv::cuda::Stream> cuda_stream_;
    cv::Ptr<cv::cuda::StereoSGM> cuda_matcher_;
    cv::cuda::GpuMat cuda_left_, cuda_right_, cuda_left_gray_, cuda_right_gray_;
    cv::cuda::GpuMat cuda_fixed_, cuda_pixels_, cuda_xyz_;
#endif
};
}
