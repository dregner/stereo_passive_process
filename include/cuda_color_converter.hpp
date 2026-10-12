#pragma once

#include <memory>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#ifdef PASSIVE_HAVE_OPENCV_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif

namespace passive_stereo_capture {
// Buffers belong to the preprocessing thread and are reused across frames.
class CudaColorConverter {
public:
    void convertPair(const cv::Mat & left, const cv::Mat & right,
                     cv::Mat & left_rgb, cv::Mat & right_rgb) {
#ifdef PASSIVE_HAVE_OPENCV_CUDA
        if (!stream_) {
            if (cv::cuda::getCudaEnabledDeviceCount() <= 0)
                throw std::runtime_error("CUDA color conversion requested but no CUDA device is available");
            stream_ = std::make_unique<cv::cuda::Stream>();
        }
        raw_[0].upload(left, *stream_);
        raw_[1].upload(right, *stream_);
        // CUDA's Bayer channel order differs from the CPU implementation in 4.10.
        // This code matches CPU COLOR_BayerRG2RGB (verified on both channels).
        for (int i = 0; i < 2; ++i)
            cv::cuda::demosaicing(raw_[i], rgb_[i], cv::COLOR_BayerRG2BGR, 3, *stream_);
        rgb_[0].download(left_rgb, *stream_);
        rgb_[1].download(right_rgb, *stream_);
        stream_->waitForCompletion();
#else
        (void)left; (void)right; (void)left_rgb; (void)right_rgb;
        throw std::runtime_error("CUDA color conversion requires OpenCV CUDA modules");
#endif
    }
private:
#ifdef PASSIVE_HAVE_OPENCV_CUDA
    std::unique_ptr<cv::cuda::Stream> stream_;
    cv::cuda::GpuMat raw_[2], rgb_[2];
#endif
};
}
