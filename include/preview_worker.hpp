#pragma once

#include <thread>
#include <atomic>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "stereo_frame.hpp"
#include "bounded_queue.hpp"

namespace passive_stereo_capture
{

/// Resizes and JPEG-compresses the stereo pair for bandwidth-limited preview streaming.
/// Publishes on:
///   left/preview/image/compressed   (CompressedImage)
///   right/preview/image/compressed  (CompressedImage)
class PreviewWorker
{
public:
    using Publisher = rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr;

    PreviewWorker(
        Publisher pub_left,
        Publisher pub_right,
        int preview_width,
        int preview_height,
        int jpeg_quality);

    ~PreviewWorker();

    /// Push a new stereo frame. Drops oldest if queue is full (best-effort).
    void push(StereoFramePtr frame);

    void start();
    void stop();

private:
    void run();

    Publisher pub_left_;
    Publisher pub_right_;
    int preview_width_;
    int preview_height_;
    int jpeg_quality_;

    BoundedQueue<StereoFramePtr> queue_{1, /*drop_oldest=*/true};
    std::thread thread_;
    std::atomic<bool> running_{false};
};

}  // namespace passive_stereo_capture
