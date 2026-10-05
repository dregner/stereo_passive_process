#pragma once

#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "spinnaker_grabber.hpp"
#include "stereo_calib.hpp"
#include "slam_worker.hpp"
#include "disparity_worker.hpp"
#include "preview_worker.hpp"
#include "gpio_trigger.hpp"
#include "clahe_processor.hpp"
#include "bounded_queue.hpp"

#include <opencv2/imgproc.hpp>

namespace passive_stereo_capture
{

/// Top-level ROS 2 node for the passive stereo capture pipeline.
class PassiveStereoNode : public rclcpp::Node
{
public:
    explicit PassiveStereoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
    ~PassiveStereoNode() override;

private:
    void declareParameters();
    void init();

    void onStereoFrame(const RawFrame & left, const RawFrame & right);
    void preprocessThread();
    void reportTimerCallback();

    void onSlamReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // ── Workers ────────────────────────────────────────────────────────────────
    std::unique_ptr<SpinnakerGrabber>  grabber_;
    std::unique_ptr<StereoCalib>       calib_;
    std::unique_ptr<SlamWorker>        slam_worker_;
    std::unique_ptr<DisparityWorker>   disp_worker_;
    std::unique_ptr<PreviewWorker>     prev_worker_;
    std::unique_ptr<GpioTrigger>       gpio_trigger_;

    // ── CLAHE ─────────────────────────────────────────────────────────────────
    cv::Ptr<cv::CLAHE> clahe_;

    // ── Preprocess queue ───────────────────────────────────────────────────────
    BoundedQueue<StereoFramePtr> preprocess_queue_{1, /*drop_oldest=*/true};
    std::thread           preprocess_thread_;
    std::atomic<bool>     preprocess_running_{false};

    rclcpp::TimerBase::SharedPtr report_timer_;

    // ── Services ───────────────────────────────────────────────────────────────
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

    // ── Feature flags ──────────────────────────────────────────────────────────
    bool slam_enabled_{true};
    bool disp_enabled_{true};
    bool prev_enabled_{true};
    bool trigger_enabled_{false};
};

}  // namespace passive_stereo_capture
