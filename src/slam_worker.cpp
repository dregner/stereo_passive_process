#include "slam_worker.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <mutex>
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
    const Config & cfg,
    ImagePub pub_tracked_image)
: node_(node),
  pub_pose_(std::move(pub_pose)),
  pub_cloud_(std::move(pub_cloud)),
  pub_path_(std::move(pub_path)),
  pub_tracked_image_(std::move(pub_tracked_image)),
  cfg_(cfg)
{
    tf_buffer_      = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_    = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    slam_.reset(new ORB_SLAM3::System(
        cfg_.voc_file,
        cfg_.settings_file,
        ORB_SLAM3::System::STEREO,
        false));  // Run without the ORB-SLAM3 viewer.

    path_msg_.header.frame_id = cfg_.frame_id;

    RCLCPP_INFO(node_->get_logger(), "SlamWorker: ORB-SLAM3 initialised");
}

SlamWorker::~SlamWorker()
{
    stop();
}

void SlamWorker::push(StereoFramePtr frame) { queue_.push(std::move(frame)); }

void SlamWorker::start()
{
    if (running_.load()) return;
    pub_cloud_.start();
    pub_path_.start();
    running_.store(true);
    thread_ = std::thread(&SlamWorker::run, this);
}

void SlamWorker::stop()
{
    running_.store(false);
    queue_.shutdown();
    if (thread_.joinable()) thread_.join();
    pub_cloud_.stop();
    pub_path_.stop();
}

void SlamWorker::reset()
{
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
            "SlamWorker: waiting for camera-to-base TF [%s -> %s]: %s",
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
    if (cfg_.enu_publish) {
        Rm = kOrbToRosEnu * Rm;// * kOrbToRosEnu.transpose();
        tv = kOrbToRosEnu * tv;
    }
    return tf2::Transform(Rm, tv);
}

void SlamWorker::publishPose(
    const Sophus::SE3f & se3, const rclcpp::Time & stamp, int state)
{
    tf2::Transform T_map_cam = sophusToTf(se3.inverse());

    tf2::Transform T_map_body = T_map_cam;
    tf2::Transform T_vision = T_map_cam;
    if (tf_cached_) {
        T_map_body = T_map_cam * T_base_cam_.inverse();

        if (!initial_offset_set_) {
            initial_offset_.setIdentity();
            initial_offset_.setOrigin(T_map_body.getOrigin());
            initial_offset_set_ = true;
        }
        T_vision = initial_offset_.inverse() * T_map_body;
    }

    const auto usec = stamp.nanoseconds() / 1000;
    if (cfg_.vision_sink && tf_cached_ && cfg_.enu_publish && usec > 0) {
        VisionSample sample;
        sample.usec = static_cast<uint64_t>(usec);
        const auto & p = T_vision.getOrigin();
        const auto q = T_vision.getRotation();
        sample.position = Eigen::Vector3d(p.x(), p.y(), p.z());
        sample.orientation = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
        sample.epoch = vision_epoch_;
        cfg_.vision_sink(std::move(sample));
    }

    tf2::Transform T_output = T_vision;
    if (cfg_.ekf_pose_at) {
        if (!tf_cached_ || !map_origin_ || usec <= 0) return;
        auto alignment = map_alignments_.find(*map_origin_);
        if (alignment == map_alignments_.end()) {
            const auto pose = cfg_.ekf_pose_at(static_cast<uint64_t>(usec));
            if (!pose) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                    "SlamWorker: map origin %lu awaiting synchronized, healthy MAVLink EKF pose",
                    *map_origin_);
                return;
            }
            const Eigen::Quaterniond q(pose->linear());
            const auto p = pose->translation();
            const tf2::Transform T_enu_body(
                tf2::Quaternion(q.x(), q.y(), q.z(), q.w()), tf2::Vector3(p.x(), p.y(), p.z()));
            const auto T_enu_map = T_enu_body * T_map_body.inverse();
            alignment = map_alignments_.emplace(*map_origin_, T_enu_map).first;
            const auto & t = T_enu_map.getOrigin();
            RCLCPP_INFO(node_->get_logger(),
                "SlamWorker: aligned map origin %lu to EKF ENU (translation %.2f, %.2f, %.2f m)",
                *map_origin_, t.x(), t.y(), t.z());
        }
        T_output = alignment->second * T_map_body;
    }
    if (cfg_.tf_publish) {
        if(!tf_cached_) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "SlamWorker: TF [%s -> %s] not yet available; publishing pose in "
                "camera frame until TF arrives",
                cfg_.parent_frame_id.c_str(), cfg_.child_frame_id.c_str());
        }
        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp    = stamp;
        ts.header.frame_id = cfg_.frame_id;
        ts.child_frame_id  = tf_cached_ ? cfg_.parent_frame_id : cfg_.child_frame_id;
        tf2::toMsg(T_output, ts.transform);
        tf_broadcaster_->sendTransform(ts);
    }

    auto msg = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg->header.stamp    = stamp;
    msg->header.frame_id = cfg_.frame_id;
    tf2::toMsg(T_output, msg->pose.pose);

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

    if (pub_pose_) {
        pub_pose_->publish(*msg);
    }

    // Accumulate path and publish if subscribed
    if (cfg_.path_pub_hz > 0.0 && pub_path_.hasSubscribers()) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = msg->header;
        ps.pose = msg->pose.pose;
        path_msg_.header.stamp = stamp;
        path_msg_.poses.push_back(ps);
        if (path_msg_.poses.size() > 5000) {
            path_msg_.poses.erase(path_msg_.poses.begin(), path_msg_.poses.begin() + 1000);
        }
        if (path_limiter_.ready(stamp.seconds(), cfg_.path_pub_hz)) {
            pub_path_.push(std::make_unique<nav_msgs::msg::Path>(path_msg_));
        }
    }
}

void SlamWorker::publishCloud(const rclcpp::Time & stamp, ORB_SLAM3::Map * map)
{
    tf2::Transform T_output_map = tf2::Transform::getIdentity();
    std::unique_lock<std::mutex> map_lock;
    if (cfg_.ekf_pose_at) {
        if (!map || !map_origin_) return;
        const auto alignment = map_alignments_.find(*map_origin_);
        if (alignment == map_alignments_.end()) return;
        map_lock = std::unique_lock(map->mMutexMapUpdate, std::try_to_lock);
        if (!map_lock || map->GetInitKFid() != *map_origin_) return;
        T_output_map = alignment->second;
    } else if (initial_offset_set_) {
        T_output_map = initial_offset_.inverse();
    }
    auto pts = slam_->GetAllMapPoints();
    std::vector<std::array<float,3>> valid_pts;
    valid_pts.reserve(pts.size());

    for (auto * p : pts) {
        if (!p || p->isBad() || (map && p->GetMap() != map)) continue;
        Eigen::Vector3f wp = p->GetWorldPos();
        tf2::Vector3 pt_orb(wp(0), wp(1), wp(2));
        tf2::Vector3 pt_ros = T_output_map * (cfg_.enu_publish ? kOrbToRosEnu * pt_orb : pt_orb);
        valid_pts.push_back({static_cast<float>(pt_ros.x()),
                             static_cast<float>(pt_ros.y()),
                             static_cast<float>(pt_ros.z())});
    }
    if (map_lock) map_lock.unlock();
    if (map && valid_pts.empty()) return;

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
    pub_cloud_.push(std::move(cloud));
}

void SlamWorker::publishTrackedImage(const rclcpp::Time & stamp)
{
    if (!pub_tracked_image_ || pub_tracked_image_->get_subscription_count() == 0) return;
    if (!tracked_image_limiter_.ready(stamp.seconds(), cfg_.tracked_image_hz)) return;

    // ORB's drawer uses the actual tracking image (including internal rectification
    // and resizing) and excludes outliers. Raw input pixels can be misaligned.
    try {
        const cv::Mat image = slam_->GetTrackingImage();
        if (image.empty()) return;
        sensor_msgs::msg::CompressedImage msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = cfg_.child_frame_id;
        msg.format = "bgr8; jpeg compressed bgr8";
        if (cv::imencode(".jpg", image, msg.data, {cv::IMWRITE_JPEG_QUALITY, 80})) {
            pub_tracked_image_->publish(std::move(msg));
        }
    } catch (const cv::Exception & ex) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "SLAM tracked image encoding failed: %s", ex.what());
    }
}

void SlamWorker::run()
{
    std::ofstream trace;
    if (!cfg_.trace_path.empty()) {
        trace.open(cfg_.trace_path);
        if (!trace) RCLCPP_WARN(node_->get_logger(), "Cannot open SLAM trace file");
        else trace << "frame_id,stamp_ns,frame_gap,queue_ms,track_ms,total_ms,state\n";
    }
    uint64_t previous_frame_id = 0;
    bool have_previous_frame = false;
    while (running_.load()) {
        StereoFramePtr frame;
        if (!queue_.pop(frame)) continue;
        const auto processing_start = std::chrono::steady_clock::now();

        if (reset_requested_.exchange(false)) {
            slam_->Reset();
            slam_->ResetActiveMap();
            initial_offset_set_ = false;
            ++vision_epoch_;
            map_origin_.reset();
            map_alignments_.clear();
            path_msg_.poses.clear();
            RCLCPP_INFO(node_->get_logger(), "SlamWorker: SLAM reset applied");
        }

        tryLookupTf();

        cv::Mat left_gray, right_gray;
        cv::cvtColor(frame->left_raw,  left_gray,  cv::COLOR_BayerRG2GRAY);
        cv::cvtColor(frame->right_raw, right_gray, cv::COLOR_BayerRG2GRAY);


        cv::Mat left_small, right_small;
        if (cfg_.scale_factor != 1.0 && cfg_.scale_factor > 0.0) {
            cv::resize(left_gray,  left_small,  cv::Size(), cfg_.scale_factor, cfg_.scale_factor, cv::INTER_LINEAR);
            cv::resize(right_gray, right_small, cv::Size(), cfg_.scale_factor, cfg_.scale_factor, cv::INTER_LINEAR);
        } else {
            left_small  = left_gray;
            right_small = right_gray;
        }

        if (cfg_.clahe_enabled) {
            clahe_gray_->apply(left_small,  left_small);
            clahe_gray_->apply(right_small, right_small);
        }

        const auto tracking_start = std::chrono::steady_clock::now();
        auto se3  = slam_->TrackStereo(left_small, right_small, frame->timestamp_sec);
        const auto tracking_finish = std::chrono::steady_clock::now();
        int state = slam_->GetTrackingState();
        last_tracking_state_.store(state);
        processed_frames_++;
        publishTrackedImage(frame->stamp);

        bool tracking = state == 2 || state == 5;
        ORB_SLAM3::Map * map = nullptr;
        if (tracking && cfg_.ekf_pose_at) {
            for (auto * point : slam_->GetTrackedMapPoints()) {
                if (!point || point->isBad()) continue;
                map = point->GetMap();
                if (map) break;
            }
            if (map) {
                const auto origin = map->GetInitKFid();
                const int big_change = map->GetLastBigChangeIdx();
                if (map_origin_ && *map_origin_ != origin) {
                    ++vision_epoch_;
                    // A merge can switch maps while TrackStereo returns. A just
                    // initialized stereo map has one keyframe and a valid pose.
                    tracking = map->KeyFramesInMap() == 1;
                    if (map_alignments_.contains(origin)) {
                        path_msg_.poses.clear();
                        RCLCPP_INFO(node_->get_logger(),
                            "SlamWorker: returning to aligned map origin %lu", origin);
                    }
                } else if (map_origin_ && big_change != map_big_change_) {
                    ++vision_epoch_;
                }
                map_origin_ = origin;
                map_big_change_ = big_change;
            } else {
                tracking = false;
            }
        }
        if (!tracking && vision_tracking_) ++vision_epoch_;
        vision_tracking_ = tracking;
        if (tracking) {
            publishPose(se3, frame->stamp, state);
        }

        // Throttle map cloud publishing & check subscriptions
        bool pub_cloud_now = false;
        if (pub_cloud_.hasSubscribers()) {
            if (!last_cloud_pub_init_) {
                pub_cloud_now = true;
                last_cloud_pub_init_ = true;
                last_cloud_pub_ = frame->stamp;
            } else if ((frame->stamp - last_cloud_pub_).seconds() >= (1.0 / std::max(0.1, cfg_.cloud_pub_hz))) {
                pub_cloud_now = true;
                last_cloud_pub_ = frame->stamp;
            }
        }
        if (pub_cloud_now && (tracking || !cfg_.ekf_pose_at)) {
            publishCloud(frame->stamp, map);
        }
        metrics_.record(processing_start, frame->received_at);
        if (trace) {
            const auto ms = [](auto a, auto b) {
                return std::chrono::duration<double, std::milli>(b-a).count();
            };
            const auto gap = have_previous_frame && frame->frame_id >= previous_frame_id
                ? frame->frame_id - previous_frame_id : 0;
            trace << frame->frame_id << ',' << frame->stamp.nanoseconds() << ',' << gap << ','
                  << ms(frame->received_at, processing_start) << ','
                  << ms(tracking_start, tracking_finish) << ','
                  << ms(processing_start, std::chrono::steady_clock::now()) << ',' << state << '\n';
        }
        previous_frame_id = frame->frame_id;
        have_previous_frame = true;
    }
}

}  // namespace passive_stereo_capture
