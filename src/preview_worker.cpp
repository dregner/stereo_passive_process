#include "preview_worker.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace passive_stereo_capture
{

PreviewWorker::PreviewWorker(
    Publisher pub_left,
    Publisher pub_right,
    int preview_width,
    int preview_height,
    int jpeg_quality)
: pub_left_(std::move(pub_left)),
  pub_right_(std::move(pub_right)),
  preview_width_(preview_width),
  preview_height_(preview_height),
  jpeg_quality_(jpeg_quality)
{}

PreviewWorker::~PreviewWorker() { stop(); }

void PreviewWorker::push(StereoFramePtr frame)
{
    queue_.push(std::move(frame));
}

void PreviewWorker::start()
{
    if (running_.load()) return;
    running_.store(true);
    thread_ = std::thread(&PreviewWorker::run, this);
}

void PreviewWorker::stop()
{
    running_.store(false);
    queue_.shutdown();
    if (thread_.joinable()) thread_.join();
}

static sensor_msgs::msg::CompressedImage encodeJpeg(
    const cv::Mat & rgb_img,
    int width, int height, int quality,
    const rclcpp::Time & stamp,
    const std::string & frame_id)
{
    // Convert RGB8 → BGR8 for OpenCV imencode
    cv::Mat bgr;
    cv::cvtColor(rgb_img, bgr, cv::COLOR_RGB2BGR);

    // Resize
    cv::Mat resized;
    if (bgr.cols != width || bgr.rows != height) {
        cv::resize(bgr, resized, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
    } else {
        resized = bgr;
    }

    // JPEG encode
    std::vector<uchar> buf;
    cv::imencode(".jpg", resized, buf,
                 {cv::IMWRITE_JPEG_QUALITY, quality});

    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = frame_id;
    msg.format          = "jpeg";
    msg.data            = std::move(buf);
    return msg;
}

void PreviewWorker::run()
{
    while (running_.load()) {
        StereoFramePtr frame;
        if (!queue_.pop(frame)) continue;

        auto left_msg  = encodeJpeg(frame->left,  preview_width_, preview_height_,
                                    jpeg_quality_, frame->stamp, "left_camera_link");
        auto right_msg = encodeJpeg(frame->right, preview_width_, preview_height_,
                                    jpeg_quality_, frame->stamp, "right_camera_link");

        pub_left_->publish(left_msg);
        pub_right_->publish(right_msg);
    }
}

}  // namespace passive_stereo_capture
