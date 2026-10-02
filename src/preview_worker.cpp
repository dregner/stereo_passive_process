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
    int jpeg_quality,
    double max_fps)
: pub_left_(std::move(pub_left)),
  pub_right_(std::move(pub_right)),
  preview_width_(preview_width),
  preview_height_(preview_height),
  jpeg_quality_(jpeg_quality),
  max_fps_(max_fps)
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
    const cv::Mat & bgr_img,
    Publisher & pub,
    std::vector<uchar> & buf,
    const rclcpp::Time & stamp,
    const std::string & frame_id)
{
    if (bgr_img.empty()) return;

    // Resize if target dimensions differ
    cv::Mat resized;
    if (bgr_img.cols != preview_width_ || bgr_img.rows != preview_height_) {
        cv::resize(bgr_img, resized, cv::Size(preview_width_, preview_height_),
                   0, 0, cv::INTER_LINEAR);
    } else {
        resized = bgr_img;
    }

    // Input is already BGR8 (the standard ROS 2 color space).
    // cv::imencode expects BGR order by default, so we encode directly with zero conversion.
    cv::imencode(".jpg", resized, buf, {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_});

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
        if (max_fps_ > 0.0) {
            if (!last_pub_time_init_) {
                last_pub_time_init_ = true;
                last_pub_time_ = frame->stamp;
            } else {
                double elapsed = (frame->stamp - last_pub_time_).seconds();
                if (elapsed < (1.0 / max_fps_)) {
                    continue; // Skip encoding and publishing this frame
                }
                last_pub_time_ = frame->stamp;
            }
        }

        // Preview publishes unrectified (raw perspective) BGR8 images directly
        encodeAndPublish(frame->left_raw_bgr,  pub_left_,  buf_left_,
                         frame->stamp, "Passive/left_camera_link");
        encodeAndPublish(frame->right_raw_bgr, pub_right_, buf_right_,
                         frame->stamp, "Passive/right_camera_link");
    }
}

}  // namespace passive_stereo_capture
