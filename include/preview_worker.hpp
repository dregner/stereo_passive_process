#pragma once

#include <thread>
#include <atomic>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include "clahe_processor.hpp"
#include "stereo_frame.hpp"
#include "bounded_queue.hpp"
#include "rate_limiter.hpp"

namespace passive_stereo_capture
{

/// Resizes and JPEG-compresses the stereo pair for bandwidth-limited preview streaming.
/// Publishes on:
///   left/preview/image/compressed   (CompressedImage)
///   right/preview/image/compressed  (CompressedImage)
class PreviewWorker
{
public:
    struct Config {
        int    preview_width{640};
        int    preview_height{480};
        int    jpeg_quality{80};
        double max_fps{10.0};
        double clahe_clip{2.0};    ///< Grayscale CLAHE clip limit (applied after resize)
        int    clahe_tiles{8};     ///< Grayscale CLAHE tile grid size
        bool   clahe_enabled{false}; ///< Apply CLAHE to preview images (Bayer→BGR8)
    };
    using Publisher = rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr;

    PreviewWorker(
        Publisher pub_left,
        Publisher pub_right,
        const Config & cfg);

    ~PreviewWorker();

    /// Push a new stereo frame. Drops oldest if queue is full (best-effort).
    void push(StereoFramePtr frame);

    void start();
    void stop();

private:
    void run();
    void encodeAndPublish(
        const cv::Mat & bgr_img,
        Publisher & pub,
        std::vector<uchar> & buf,
        const rclcpp::Time & stamp,
        const std::string & frame_id);

    Publisher pub_left_;
    Publisher pub_right_;
    Config cfg_;

    cv::Ptr<cv::CLAHE> clahe_{cv::createCLAHE(cfg_.clahe_clip, cv::Size(cfg_.clahe_tiles, cfg_.clahe_tiles))};

    // FIX #10: persistent encode buffers — cv::imencode reuses allocation
    std::vector<uchar> buf_left_;
    std::vector<uchar> buf_right_;

    RateLimiter limiter_;
    BoundedQueue<StereoFramePtr> queue_{1, /*drop_oldest=*/true};
    std::thread thread_;
    std::atomic<bool> running_{false};
};

}  // namespace passive_stereo_capture
