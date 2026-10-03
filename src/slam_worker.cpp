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

    slam_.reset(new ORB_SLAM3::System(
        cfg_.voc_file,
        cfg_.settings_file,
        ORB_SLAM3::System::STEREO,
        cfg_.use_pangolin));

    // Grayscale CLAHE applied after resize (small image — efficient)
    clahe_gray_ = cv::createCLAHE(cfg_.clahe_clip,
                                   cv::Size(cfg_.clahe_tiles, cfg_.clahe_tiles));

    RCLCPP_INFO(node_->get_logger(), "SlamWorker: ORB-SLAM3 initialised");
}

SlamWorker::~SlamWorker()
{
    stop();
    // unique_ptr destructor calls SlamDeleter → Shutdown() + delete
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
    // Safe: just set atomic flag; run() applies it at the top of its loop
    reset_requested_.store(true);
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
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "SlamWorker: TF [%s -> %s] not yet available; publishing pose in "
            "camera frame until TF arrives: %s",
            cfg_.parent_frame_id.c_str(), cfg_.child_frame_id.c_str(), ex.what());
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
    const Sophus::SE3f & se3, const rclcpp::Time & stamp, int state)
{
    tf2::Transform T_map_cam = sophusToTf(se3.inverse());

    tf2::Transform T_zeroed;
    if (tf_cached_) {
        tf2::Transform T_cam_base = T_base_cam_.inverse();
        tf2::Transform T_map_base = T_map_cam * T_cam_base;

        if (!initial_offset_set_) {
            initial_offset_.setIdentity();
            initial_offset_.setOrigin(T_map_base.getOrigin());
            initial_offset_set_ = true;
        }
        T_zeroed = initial_offset_.inverse() * T_map_base;
    } else {
        // TF not yet available — publish raw ENU pose in map frame
        if (!initial_offset_set_) {
            initial_offset_.setIdentity();
            initial_offset_.setOrigin(T_map_cam.getOrigin());
            initial_offset_set_ = true;
        }
        T_zeroed = initial_offset_.inverse() * T_map_cam;
    }

    if (cfg_.tf_publish) {
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp    = stamp;
        ts.header.frame_id = cfg_.frame_id;
        ts.child_frame_id  = tf_cached_ ? cfg_.parent_frame_id : cfg_.child_frame_id;
        tf2::toMsg(T_zeroed, ts.transform);
        tf_broadcaster_->sendTransform(ts);
    }

    auto msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg->header.stamp    = stamp;
    msg->header.frame_id = cfg_.frame_id;
    tf2::toMsg(T_zeroed, msg->pose.pose);

 
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
    std::vector<std::array<float,3>> valid_pts;
    valid_pts.reserve(pts.size());

    for (auto * p : pts) {
        if (!p || p->isBad()) continue;
        Eigen::Vector3f wp = p->GetWorldPos();
        tf2::Vector3 pt_orb(wp(0), wp(1), wp(2));
        tf2::Vector3 pt_ros = kOrbToRosEnu * pt_orb;
        if (tf_cached_) pt_ros = pt_ros + T_base_cam_.getOrigin();
        valid_pts.push_back({static_cast<float>(pt_ros.x()),
                             static_cast<float>(pt_ros.y()),
                             static_cast<float>(pt_ros.z())});
    }

    if (valid_pts.empty()) return;

    auto cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    cloud->header.stamp    = stamp;
    cloud->header.frame_id = cfg_.frame_id;
    cloud->height = 1;
    cloud->width  = static_cast<uint32_t>(valid_pts.size());
    cloud->is_dense = true;
    cloud->is_bigendian = false;
    cloud->fields.resize(3);
    for (int i = 0; i < 3; ++i) {
        cloud->fields[i].name     = std::string(1, "xyz"[i]);
        cloud->fields[i].offset   = i * 4;
        cloud->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud->fields[i].count    = 1;
    }
    cloud->point_step = 12;
    cloud->row_step   = 12 * cloud->width;
    cloud->data.resize(cloud->row_step);
    std::memcpy(cloud->data.data(), valid_pts.data(), cloud->row_step);
    pub_cloud_->publish(std::move(cloud));
}

void SlamWorker::run()
{
    while (running_.load()) {
        StereoFramePtr frame;
        if (!queue_.pop(frame)) continue;

        // Handle reset safely inside the worker thread (atomic flag set by reset())
        if (reset_requested_.exchange(false)) {
            slam_->Reset();
            slam_->ResetActiveMap();
            initial_offset_set_ = false;
            RCLCPP_INFO(node_->get_logger(), "SlamWorker: SLAM reset applied");
        }

        tryLookupTf();

        // SLAM path: receives rectified grayscale (CV_8UC1).
        // BayerRG2GRAY already done in preprocessThread — no color conversion needed here.
        // Apply scale_factor directly via cv::resize fractional scaling.
        cv::Mat left_small, right_small;
        if (cfg_.scale_factor != 1.0 && cfg_.scale_factor > 0.0) {
            cv::resize(frame->left_raw,  left_small,  cv::Size(), cfg_.scale_factor, cfg_.scale_factor, cv::INTER_LINEAR);
            cv::resize(frame->right_raw, right_small, cv::Size(), cfg_.scale_factor, cfg_.scale_factor, cv::INTER_LINEAR);
        } else {
            left_small  = frame->left_raw;
            right_small = frame->right_raw;
        }

        // Apply CLAHE on the small grayscale image — efficient (small resolution)
        clahe_gray_->apply(left_small,  left_small);
        clahe_gray_->apply(right_small, right_small);

        // Track
        auto se3  = slam_->TrackStereo(left_small, right_small, frame->timestamp_sec);
        int state = slam_->GetTrackingState();

        // Only publish pose when tracking is valid (state 2=OK, 5=RECENTLY_LOST with pose)
        if (state == 2 || state == 5) {
            publishPose(se3, frame->stamp, state);
        }

        // Throttle map cloud (GetAllMapPoints acquires internal ORB-SLAM3 mutex)
        bool pub_cloud_now = false;
        if (!last_cloud_pub_init_) {
            pub_cloud_now = true;
            last_cloud_pub_init_ = true;
            last_cloud_pub_ = frame->stamp;
        } else if ((frame->stamp - last_cloud_pub_).seconds() >= (1.0 / cfg_.cloud_pub_hz)) {
            pub_cloud_now = true;
            last_cloud_pub_ = frame->stamp;
        }
        if (pub_cloud_now) publishCloud(frame->stamp);
    }
}

}  // namespace passive_stereo_capture
