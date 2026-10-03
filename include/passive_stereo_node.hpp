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
///
/// Responsibilities:
///   - Declare and manage all parameters
///   - Open Spinnaker cameras via SpinnakerGrabber
///   - Non-blocking callback from sync thread → push raw pair to preprocess queue
///   - Dedicated preprocess thread: debayer + rectify + CLAHE → dispatch to workers
///   - Optionally generate hardware trigger via GpioTrigger
class PassiveStereoNode : public rclcpp::Node
{
public:
    explicit PassiveStereoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});
    ~PassiveStereoNode() override;

private:
    void declareParameters();
    void init();

    // Sync-thread callback: non-blocking, pushes raw pair to preprocess queue
    void onStereoFrame(const RawFrame & left, const RawFrame & right);

    // FIX #12: preprocess thread — dequeues raw pairs, rectifies, applies CLAHE,
    // dispatches StereoFrame to workers (off the sync thread critical path)
    void preprocessThread();

    void onSlamReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // ── Workers ────────────────────────────────────────────────────────────────
    std::unique_ptr<SpinnakerGrabber>  grabber_;
    std::unique_ptr<StereoCalib>   calib_;
    std::unique_ptr<SlamWorker>        slam_worker_;
    std::unique_ptr<DisparityWorker>   disp_worker_;
    std::unique_ptr<PreviewWorker>     prev_worker_;
    std::unique_ptr<GpioTrigger>       gpio_trigger_;

    // ── CLAHE (full-res, for Retinify/Preview) ─────────────────────────────────
    cv::Ptr<cv::CLAHE> clahe_;

    // ── Preprocess queue (raw frame pairs, between sync thread and preprocess thread) ──
    using RawPair = std::pair<RawFrame, RawFrame>;
    BoundedQueue<RawPair> preprocess_queue_{2, /*drop_oldest=*/true};
    std::thread           preprocess_thread_;
    std::atomic<bool>     preprocess_running_{false};

    // ── Services ───────────────────────────────────────────────────────────────
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

    // ── Feature flags (read once at init) ─────────────────────────────────────
    bool slam_enabled_{true};
    bool disp_enabled_{true};
    bool prev_enabled_{true};
    bool trigger_enabled_{false};
};

}  // namespace passive_stereo_capture
