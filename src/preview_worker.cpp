#include "preview_worker.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace passive_stereo_capture
{

PreviewWorker::PreviewWorker(
    Publisher pub_left,
    Publisher pub_right,
    const Config & cfg)
: pub_left_(std::move(pub_left)),
  pub_right_(std::move(pub_right)),
  cfg_(cfg)
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
    if (rgb_img.empty() || !pub || pub->get_subscription_count() == 0) return;

    cv::Mat resized;
    if (rgb_img.cols != cfg_.preview_width || rgb_img.rows != cfg_.preview_height) {
        cv::resize(rgb_img, resized, cv::Size(cfg_.preview_width, cfg_.preview_height),
                   0, 0, cv::INTER_LINEAR);
    } else {
        resized = rgb_img;
    }

    cv::Mat bgr;
    if (cfg_.clahe_enabled) {
        bgr = applyClaheBGR(resized, clahe_);
    } else {
        // cv::cvtColor(resized, bgr, cv::COLOR_RGB2BGR);
        bgr = resized;  // No color conversion needed to show in foxglove, which expects RGB images.
    }

    cv::imencode(".jpg", bgr, buf, {cv::IMWRITE_JPEG_QUALITY, cfg_.jpeg_quality});

    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp    = stamp;
    msg.header.frame_id = frame_id;
    msg.format          = "jpeg";
    msg.data            = buf;
    pub->publish(msg);
}

void PreviewWorker::run()
{
    while (running_.load()) {
        StereoFramePtr frame;
        if (!queue_.pop(frame)) continue;

        bool has_left_sub  = (pub_left_ && pub_left_->get_subscription_count() > 0);
        bool has_right_sub = (pub_right_ && pub_right_->get_subscription_count() > 0);

        // If no one is listening to preview, skip heavy resize and JPEG compression completely!
        if (!has_left_sub && !has_right_sub) {
            continue;
        }

        if (cfg_.max_fps > 0.0 && !limiter_.ready(frame->stamp.seconds(), cfg_.max_fps)) continue;

        if (has_left_sub) {
            encodeAndPublish(frame->left_rgb,   pub_left_,  buf_left_,
                            frame->stamp, "Passive/left_camera_link");
        }
        if (has_right_sub) {
            encodeAndPublish(frame->right_rgb, pub_right_, buf_right_,
                            frame->stamp, "Passive/right_camera_link");
        }
    }
}

}  // namespace passive_stereo_capture
