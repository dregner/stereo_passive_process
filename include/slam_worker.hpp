#pragma once

#include <thread>
#include <atomic>
#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>

// Prevent Pangolin Xlib collision with ROS headers
#pragma push_macro("None")
#undef None
#include <System.h>
#pragma pop_macro("None")

#include <sophus/se3.hpp>
#include <opencv2/imgproc.hpp>

#include "stereo_frame.hpp"
#include "bounded_queue.hpp"
#include "worker_metrics.hpp"

namespace passive_stereo_capture
{

/// Wraps ORB-SLAM3 stereo tracking in a dedicated thread.
///
/// Input: StereoFrame::left_raw / right_raw (BayerRG8, converted directly to GRAY).
/// SLAM resizes using scale_factor (or keeps 1.0 if width <= 800) and applies grayscale CLAHE.
class SlamWorker
{
public:
    struct Config {
        std::string voc_file;
        std::string settings_file;
        bool use_pangolin{false};
        double scale_factor{1.0};
        std::string frame_id{"map"};
        std::string parent_frame_id{"base_link"};
        std::string child_frame_id{"Passive/left_camera_link"};
        bool enu_publish{true};
        bool tf_publish{false};
        double cloud_pub_hz{2.0};
        double clahe_clip{2.0};
        int    clahe_tiles{8};
        bool   clahe_enabled{false};
    };

    using PosePub   = rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr;
    using CloudPub  = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;
    using PathPub   = rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr;

    SlamWorker(
        rclcpp::Node * node,
        PosePub  pub_pose,
        CloudPub pub_cloud,
        PathPub  pub_path,
        const Config & cfg);

    ~SlamWorker();

    void push(StereoFramePtr frame);
    void start();
    void stop();

    /// Thread-safe reset: applied at the top of the next run() iteration.
    void reset();

    int lastTrackingState() const { return last_tracking_state_.load(); }
    uint64_t processedFrames() const { return processed_frames_.load(); }

    uint64_t droppedFrames() const { return queue_.dropped(); }
    double processingMs() const { return metrics_.process_ms.load(); }
    double receiptAgeMs() const { return metrics_.receipt_age_ms.load(); }

private:
    void run();
    void publishPose(const Sophus::SE3f & se3, const rclcpp::Time & stamp, int tracking_state);
    void publishCloud(const rclcpp::Time & stamp);
    tf2::Transform sophusToTf(const Sophus::SE3f & pose);
    void tryLookupTf();

    static const tf2::Matrix3x3 kOrbToRosEnu;

    rclcpp::Node * node_;
    PosePub  pub_pose_;
    CloudPub pub_cloud_;
    PathPub  pub_path_;
    Config   cfg_;

    std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    struct SlamDeleter {
        void operator()(ORB_SLAM3::System * s) const {
            if (s) { s->Shutdown(); delete s; }
        }
    };
    std::unique_ptr<ORB_SLAM3::System, SlamDeleter> slam_;

    tf2::Transform T_base_cam_;
    tf2::Transform initial_offset_;
    bool tf_cached_{false};
    bool initial_offset_set_{false};

    nav_msgs::msg::Path path_msg_;

    rclcpp::Time last_cloud_pub_;
    bool         last_cloud_pub_init_{false};

    std::atomic<bool> reset_requested_{false};
    std::atomic<int>  last_tracking_state_{-1};
    std::atomic<uint64_t> processed_frames_{0};

    cv::Ptr<cv::CLAHE> clahe_gray_{cv::createCLAHE(cfg_.clahe_clip, cv::Size(cfg_.clahe_tiles, cfg_.clahe_tiles))};

    BoundedQueue<StereoFramePtr> queue_{1, /*drop_oldest=*/true};
    WorkerMetrics metrics_;
    std::thread       thread_;
    std::atomic<bool> running_{false};
};

}  // namespace passive_stereo_capture
