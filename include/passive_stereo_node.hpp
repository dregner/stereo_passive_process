#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "spinnaker_grabber.hpp"
#include "stereo_rectifier.hpp"
#include "slam_worker.hpp"
#include "disparity_worker.hpp"
#include "preview_worker.hpp"
#include "gpio_trigger.hpp"
#include "clahe_processor.hpp"

#include <opencv2/imgproc.hpp>

namespace passive_stereo_capture
{

/// Top-level ROS 2 node for the ROS-free passive stereo capture pipeline.
///
/// Responsibilities:
///   - Declare and manage all parameters
///   - Open Spinnaker cameras via SpinnakerGrabber
///   - On each synchronized pair: debayer -> rectify -> CLAHE
///   - Dispatch StereoFrame to SlamWorker, DisparityWorker, PreviewWorker
///   - Optionally generate hardware trigger via GpioTrigger
class PassiveStereoNode : public rclcpp::Node
{
public:
    explicit PassiveStereoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
    ~PassiveStereoNode() override;

private:
    void declareParameters();
    void init();
    void onStereoFrame(const RawFrame & left, const RawFrame & right);
    void onSlamReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // ── Workers ────────────────────────────────────────────────────────────────
    std::unique_ptr<SpinnakerGrabber>  grabber_;
    std::unique_ptr<StereoRectifier>   rectifier_;
    std::unique_ptr<SlamWorker>        slam_worker_;
    std::unique_ptr<DisparityWorker>   disp_worker_;
    std::unique_ptr<PreviewWorker>     prev_worker_;
    std::unique_ptr<GpioTrigger>       gpio_trigger_;

    // ── CLAHE ──────────────────────────────────────────────────────────────────
    cv::Ptr<cv::CLAHE> clahe_;

    // ── Services ───────────────────────────────────────────────────────────────
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

    // ── Feature flags (read once at init) ─────────────────────────────────────
    bool slam_enabled_{true};
    bool disp_enabled_{true};
    bool prev_enabled_{true};
    bool trigger_enabled_{false};
};

}  // namespace passive_stereo_capture
