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

namespace passive_stereo_capture
{

/// Wraps ORB-SLAM3 stereo tracking in a dedicated thread.
///
/// Input: StereoFrame::left_gray / right_gray (CV_8UC1, rectified, full-res).
///   BayerRG→GRAY conversion already done in preprocessThread (single-step, efficient).
///   SLAM resizes using scale_factor and applies grayscale CLAHE.
///
/// Output: PoseWithCovarianceStamped + sparse PointCloud2 (throttled).
class SlamWorker
{
public:
    struct Config {
        std::string voc_file;
        std::string settings_file;
        bool use_pangolin{false};
        double scale_factor{1.0};  ///< Scale applied to input before TrackStereo (e.g. 0.33 for 800x600 from 2448x2048)
        std::string frame_id{"map"};
        std::string parent_frame_id{"base_link"};
        std::string child_frame_id{"Passive/left_camera_link"};
        bool enu_publish{true};
        bool tf_publish{false};
        double cloud_pub_hz{2.0};  ///< Max rate to call GetAllMapPoints (acquires mutex)
        double clahe_clip{2.0};    ///< Grayscale CLAHE clip limit (applied after resize)
        int    clahe_tiles{8};     ///< Grayscale CLAHE tile grid size
        bool  clahe_enabled{false}; ///< Apply CLAHE to preview images (Bayer→BGR8)
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

private:
    void run();
    void publishPose(const Sophus::SE3f & se3, const rclcpp::Time & stamp, int tracking_state);
    void publishCloud(const rclcpp::Time & stamp);
    tf2::Transform sophusToTf(const Sophus::SE3f & pose);
    void tryLookupTf();

    // ENU rotation: ORB-SLAM (Z-forward, X-right, Y-down) -> ROS ENU (X-forward, Y-left, Z-up)
    static const tf2::Matrix3x3 kOrbToRosEnu;

    rclcpp::Node * node_;
    PosePub  pub_pose_;
    CloudPub pub_cloud_;
    PathPub  pub_path_;
    Config   cfg_;

    std::shared_ptr<tf2_ros::Buffer>               tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener>    tf_listener_;  // kept alive to fill buffer
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Exception-safe ownership: destructor calls Shutdown() + delete
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

    // Throttle map cloud publish
    rclcpp::Time last_cloud_pub_;
    bool         last_cloud_pub_init_{false};

    // Atomic reset flag — avoids race between service thread and run() thread
    std::atomic<bool> reset_requested_{false};

    // Grayscale CLAHE applied post-resize (efficient — small image only)
    // cv::Ptr<cv::CLAHE> clahe_gray_;
    cv::Ptr<cv::CLAHE> clahe_gray_{cv::createCLAHE(cfg_.clahe_clip, cv::Size(cfg_.clahe_tiles, cfg_.clahe_tiles))};

    BoundedQueue<StereoFramePtr> queue_{2, /*drop_oldest=*/false};
    std::thread       thread_;
    std::atomic<bool> running_{false};
};

}  // namespace passive_stereo_capture
