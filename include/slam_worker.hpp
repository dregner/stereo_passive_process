#pragma once

#include <thread>
#include <atomic>
#include <memory>
#include <string>

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

#include "stereo_frame.hpp"
#include "bounded_queue.hpp"

namespace passive_stereo_capture
{

/// Wraps ORB-SLAM3 stereo tracking in a dedicated thread.
/// Converts RGB8 frames to MONO8, resizes to fit within max_width x max_height,
/// calls TrackStereo() and publishes pose + sparse map pointcloud to ROS 2.
class SlamWorker
{
public:
    struct Config {
        std::string voc_file;
        std::string settings_file;
        bool use_pangolin{false};
        int  max_width{800};   ///< Maximum width for SLAM input image
        int  max_height{600};  ///< Maximum height for SLAM input image
        std::string frame_id{"map"};
        std::string parent_frame_id{"base_link"};
        std::string child_frame_id{"Passive/left_camera_link"};
        bool enu_publish{true};
        bool tf_publish{false};
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
    void reset();

private:
    void run();
    void publishPose(const Sophus::SE3f & se3, const rclcpp::Time & stamp);
    void publishCloud(const rclcpp::Time & stamp);
    tf2::Transform sophusToTf(const Sophus::SE3f & pose);
    void tryLookupTf();

    // ENU rotation: ORB-SLAM (Z-forward, X-right, Y-down) -> ROS ENU
    static const tf2::Matrix3x3 kOrbToRosEnu;

    rclcpp::Node * node_;
    PosePub  pub_pose_;
    CloudPub pub_cloud_;
    PathPub  pub_path_;
    Config   cfg_;

    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    ORB_SLAM3::System * slam_{nullptr};

    tf2::Transform T_base_cam_;
    tf2::Transform initial_offset_;
    bool tf_cached_{false};
    bool initial_offset_set_{false};

    BoundedQueue<StereoFramePtr> queue_{2, /*drop_oldest=*/false};
    std::thread       thread_;
    std::atomic<bool> running_{false};
};

}  // namespace passive_stereo_capture
