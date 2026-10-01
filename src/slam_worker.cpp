#include "slam_worker.hpp"

#include <opencv2/imgproc.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

namespace passive_stereo_capture
{

// ORB-SLAM3: Z=forward, X=right, Y=down  -->  ROS ENU: X=forward, Y=left, Z=up
const tf2::Matrix3x3 SlamWorker::kOrbToRosEnu(
     0.0,  0.0,  1.0,
    -1.0,  0.0,  0.0,
     0.0, -1.0,  0.0);

SlamWorker::SlamWorker(
    rclcpp::Node * node,
    PosePub  pub_pose,
    CloudPub pub_cloud,
    PathPub  pub_path,
    const Config & cfg)
: node_(node),
  pub_pose_(std::move(pub_pose)),
  pub_cloud_(std::move(pub_cloud)),
  pub_path_(std::move(pub_path)),
  cfg_(cfg)
{
    tf_buffer_      = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    // Initialise ORB-SLAM3
    slam_ = new ORB_SLAM3::System(
        cfg_.voc_file,
        cfg_.settings_file,
        ORB_SLAM3::System::STEREO,
        cfg_.use_pangolin);

    RCLCPP_INFO(node_->get_logger(), "SlamWorker: ORB-SLAM3 initialised");
}

SlamWorker::~SlamWorker()
{
    stop();
    if (slam_) {
        slam_->Shutdown();
        delete slam_;
    }
}

void SlamWorker::push(StereoFramePtr frame) { queue_.push(std::move(frame)); }

void SlamWorker::start()
{
    if (running_.load()) return;
    running_.store(true);
    thread_ = std::thread(&SlamWorker::run, this);
}

void SlamWorker::stop()
{
    running_.store(false);
    queue_.shutdown();
    if (thread_.joinable()) thread_.join();
}

void SlamWorker::reset()
{
    if (slam_) {
        slam_->Reset();
        slam_->ResetActiveMap();
        initial_offset_set_ = false;
    }
}

void SlamWorker::tryLookupTf()
{
    if (tf_cached_) return;
    try {
        auto t = tf_buffer_->lookupTransform(
            cfg_.parent_frame_id, cfg_.child_frame_id, tf2::TimePointZero);
        tf2::fromMsg(t.transform, T_base_cam_);
        tf_cached_ = true;
        RCLCPP_INFO(node_->get_logger(),
            "SlamWorker: TF [%s -> %s] cached",
            cfg_.parent_frame_id.c_str(), cfg_.child_frame_id.c_str());
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
            "SlamWorker: waiting for TF: %s", ex.what());
    }
}

tf2::Transform SlamWorker::sophusToTf(const Sophus::SE3f & pose)
{
    Eigen::Matrix3d R = pose.rotationMatrix().cast<double>();
    Eigen::Vector3d t = pose.translation().cast<double>();

    tf2::Matrix3x3 Rm(R(0,0),R(0,1),R(0,2),
                      R(1,0),R(1,1),R(1,2),
                      R(2,0),R(2,1),R(2,2));
    tf2::Vector3 tv(t(0), t(1), t(2));

    Rm = kOrbToRosEnu * Rm;
    tv = kOrbToRosEnu * tv;
    return tf2::Transform(Rm, tv);
}

void SlamWorker::publishPose(
    const Sophus::SE3f & se3, const rclcpp::Time & stamp)
{
    if (!tf_cached_) return;

    // map -> camera from SLAM (inverse of tracked pose)
    tf2::Transform T_map_cam   = sophusToTf(se3.inverse());
    tf2::Transform T_cam_base  = T_base_cam_.inverse();
    tf2::Transform T_map_base  = T_map_cam * T_cam_base;

    if (!initial_offset_set_) {
        initial_offset_.setIdentity();
        initial_offset_.setOrigin(T_map_base.getOrigin());
        initial_offset_set_ = true;
    }
    tf2::Transform T_zeroed = initial_offset_.inverse() * T_map_base;

    // Publish TF if requested
    if (cfg_.tf_publish) {
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp    = stamp;
        ts.header.frame_id = cfg_.frame_id;
        ts.child_frame_id  = cfg_.parent_frame_id;
        tf2::toMsg(T_zeroed, ts.transform);
        tf_broadcaster_->sendTransform(ts);
    }

    // Pose with covariance
    auto msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg->header.stamp    = stamp;
    msg->header.frame_id = cfg_.frame_id;
    tf2::toMsg(T_zeroed, msg->pose.pose);

    int state = slam_->GetTrackingState();
    std::fill(msg->pose.covariance.begin(), msg->pose.covariance.end(), 0.0);
    if (state == 2 || state == 5) {
        msg->pose.covariance[0]  = 0.05;
        msg->pose.covariance[7]  = 0.05;
        msg->pose.covariance[14] = 0.05;
        msg->pose.covariance[21] = 0.1;
        msg->pose.covariance[28] = 0.1;
        msg->pose.covariance[35] = 0.1;
    } else if (state == 3) {
        msg->pose.covariance.fill(1.0);
    } else {
        msg->pose.covariance.fill(-1.0);
    }

    pub_pose_->publish(std::move(msg));
}

void SlamWorker::publishCloud(const rclcpp::Time & stamp)
{
    auto pts = slam_->GetAllMapPoints();
    int count = 0;
    for (auto * p : pts) if (p) ++count;
    if (count == 0) return;

    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header.stamp    = stamp;
    cloud->header.frame_id = cfg_.frame_id;
    cloud->height = 1;
    cloud->width  = static_cast<uint32_t>(count);
    cloud->is_dense = true;
    cloud->fields.resize(3);
    for (int i = 0; i < 3; ++i) {
        cloud->fields[i].name     = std::string(1, "xyz"[i]);
        cloud->fields[i].offset   = i * 4;
        cloud->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud->fields[i].count    = 1;
    }
    cloud->point_step    = 12;
    cloud->row_step      = 12 * count;
    cloud->is_bigendian  = false;
    cloud->data.resize(12 * count);

    tf2::Vector3 cam_off = T_base_cam_.getOrigin();
    int idx = 0;
    for (auto * p : pts) {
        if (!p) continue;
        float x = p->GetWorldPos()(0);
        float y = p->GetWorldPos()(1);
        float z = p->GetWorldPos()(2);
        tf2::Vector3 pt_orb(x, y, z);
        tf2::Vector3 pt_ros = kOrbToRosEnu * pt_orb + cam_off;
        float fx = pt_ros.x(), fy = pt_ros.y(), fz = pt_ros.z();
        std::memcpy(&cloud->data[idx * 12 + 0], &fx, 4);
        std::memcpy(&cloud->data[idx * 12 + 4], &fy, 4);
        std::memcpy(&cloud->data[idx * 12 + 8], &fz, 4);
        ++idx;
    }
    pub_cloud_->publish(std::move(cloud));
}

void SlamWorker::run()
{
    while (running_.load()) {
        StereoFramePtr frame;
        if (!queue_.pop(frame)) continue;

        tryLookupTf();

        // Convert RGB8 -> MONO8
        cv::Mat left_mono, right_mono;
        cv::cvtColor(frame->left,  left_mono,  cv::COLOR_RGB2GRAY);
        cv::cvtColor(frame->right, right_mono, cv::COLOR_RGB2GRAY);

        // Resize to fit within max_width x max_height
        int W = left_mono.cols, H = left_mono.rows;
        double scale = std::min(
            static_cast<double>(cfg_.max_width)  / W,
            static_cast<double>(cfg_.max_height) / H);
        if (scale < 1.0) {
            int nw = static_cast<int>(W * scale);
            int nh = static_cast<int>(H * scale);
            cv::resize(left_mono,  left_mono,  cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
            cv::resize(right_mono, right_mono, cv::Size(nw, nh), 0, 0, cv::INTER_LINEAR);
        }

        // Track
        auto se3 = slam_->TrackStereo(left_mono, right_mono, frame->timestamp_sec);

        publishPose(se3, frame->stamp);
        publishCloud(frame->stamp);
    }
}

}  // namespace passive_stereo_capture
