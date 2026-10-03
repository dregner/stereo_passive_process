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
    if (rgb_img.empty()) return;

    // Resize if target dimensions differ
    
    cv::Mat resized;
    if (rgb_img.cols != cfg_.preview_width || rgb_img.rows != cfg_.preview_height) {
        cv::resize(rgb_img, resized, cv::Size(cfg_.preview_width, cfg_.preview_height),
                   0, 0, cv::INTER_LINEAR);
    } else {
        resized = rgb_img;
    }
    if (cfg_.clahe_enabled) {
        resized=applyClaheBGR(resized, clahe_);
    }
    else{
        cv::cvtColor(resized, resized, cv::COLOR_BayerRG2BGR);
    }
    // Input is already BGR8 (the standard ROS 2 color space).
    // cv::imencode expects BGR order by default, so we encode directly with zero conversion.
    cv::imencode(".jpg", resized, buf, {cv::IMWRITE_JPEG_QUALITY, cfg_.jpeg_quality});

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

        // Rate limiting: throttle preview compression and publishing to max_fps
        if (cfg_.max_fps > 0.0) {
            if (!last_pub_time_init_) {
                last_pub_time_init_ = true;
                last_pub_time_ = frame->stamp;
            } else {
                double elapsed = (frame->stamp - last_pub_time_).seconds();
                if (elapsed < (1.0 / cfg_.max_fps)) {
                    continue; // Skip encoding and publishing this frame
                }
                last_pub_time_ = frame->stamp;
            }
        }

        // Preview publishes unrectified (raw perspective) BGR8 images directly
        encodeAndPublish(frame->left_rgb,   pub_left_,  buf_left_,
                         frame->stamp, "Passive/left_camera_link");
        encodeAndPublish(frame->right_rgb, pub_right_, buf_right_,
                         frame->stamp, "Passive/right_camera_link");
    }
}

}  // namespace passive_stereo_capture
