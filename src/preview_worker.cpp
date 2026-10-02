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

void PreviewWorker::encodeAndPublish(
    const cv::Mat & rgb_img,
    Publisher & pub,
    std::vector<uchar> & buf,
    const rclcpp::Time & stamp,
    const std::string & frame_id)
{
    // FIX #10: Resize first (cheaper when smaller), then convert color.
    // Avoids a full-res RGB->BGR copy before resizing.
    cv::Mat resized;
    if (rgb_img.cols != preview_width_ || rgb_img.rows != preview_height_) {
        cv::resize(rgb_img, resized, cv::Size(preview_width_, preview_height_),
                   0, 0, cv::INTER_LINEAR);
    } else {
        resized = rgb_img;
    }

    // Convert RGB8 → BGR8 for OpenCV imencode (JPEG codec expects BGR)
    cv::Mat bgr;
    cv::cvtColor(resized, bgr, cv::COLOR_RGB2BGR);

    // FIX #10: buf is a member variable — cv::imencode reuses its allocation.
    cv::imencode(".jpg", bgr, buf, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});

    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = frame_id;
    msg.format          = "jpeg";
    msg.data            = buf;   // copy into message (ROS 2 publish owns the data)
    pub->publish(msg);
}

void PreviewWorker::run()
{
    while (running_.load()) {
        StereoFramePtr frame;
        if (!queue_.pop(frame)) continue;

        encodeAndPublish(frame->left_rgb,  pub_left_,  buf_left_,
                         frame->stamp, "Passive/left_camera_link");
        encodeAndPublish(frame->right_rgb, pub_right_, buf_right_,
                         frame->stamp, "Passive/right_camera_link");
    }
}

}  // namespace passive_stereo_capture
