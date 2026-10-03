#include "passive_stereo_node.hpp"

#include <chrono>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace passive_stereo_capture
{

PassiveStereoNode::PassiveStereoNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("passive_stereo_node", options)
{
    declareParameters();
    init();
}

PassiveStereoNode::~PassiveStereoNode()
{
    // 1. Stop grabber — no more raw pairs pushed to preprocess queue
    if (grabber_) grabber_->stop();

    // 2. Drain and stop preprocess thread — no more StereoFrames pushed to workers
    preprocess_running_.store(false);
    preprocess_queue_.shutdown();
    if (preprocess_thread_.joinable()) preprocess_thread_.join();

    // 3. Stop workers last
    if (gpio_trigger_) gpio_trigger_->stop();
    if (slam_worker_)  slam_worker_->stop();
    if (disp_worker_)  disp_worker_->stop();
    if (prev_worker_)  prev_worker_->stop();
}

void PassiveStereoNode::declareParameters()
{
    // Camera
    declare_parameter("cam_left_serial",    "22548033");
    declare_parameter("cam_right_serial",   "22548025");
    declare_parameter("frame_rate",         30.0);
    declare_parameter("exposure_time",      33333.84);
    declare_parameter("gain",               0.0);
    declare_parameter("gain_auto",          false);
    declare_parameter("balance_white_auto", true);
    declare_parameter("binning",            1);
    declare_parameter("trigger_mode",       false);
    declare_parameter("trigger_delay_us",   29);
    declare_parameter("max_consec_errors",  10);

    // GPIO
    declare_parameter("gpio_chip",         std::string("gpiochip0"));
    declare_parameter("gpio_line",         (int)85);   // PN.01 = GPIO27 = trigger line

    // Calibration
    declare_parameter("calibration_file",  std::string(""));

    // CLAHE (applied on RGB for Retinify/Preview; gray CLAHE params reused for SLAM)
    declare_parameter("clahe_clip_limit",  2.0);
    declare_parameter("clahe_tile_size",   8);
    declare_parameter("enable_clahe",       false);

    // SLAM
    declare_parameter("slam_enabled",       true);
    declare_parameter("slam_voc_file",       std::string(""));
    declare_parameter("slam_settings_file",  std::string(""));
    declare_parameter("slam_use_pangolin",   false);
    declare_parameter("slam_scale_factor",   1.0);   ///< e.g. 0.33 → ~800×680 from 2448×2048
    declare_parameter("slam_enu_publish",    true);
    declare_parameter("slam_tf_publish",     false);
    declare_parameter("slam_cloud_hz",       2.0);
    declare_parameter("frame_id",            std::string("map"));
    declare_parameter("parent_frame_id",     std::string("base_link"));
    declare_parameter("child_frame_id",      std::string("Passive/left_camera_link"));

    // Disparity
    declare_parameter("disparity_enabled",   true);
    declare_parameter("depth_mode",          std::string("accurate"));
    declare_parameter("max_dist",            15.0);
    declare_parameter("sampling_factor",     1.0);
    declare_parameter("crop_factor",         1.0);
    declare_parameter("min_confidence",      0.35);
    declare_parameter("confidence_radius",   2);
    declare_parameter("confidence_alpha",    2.0);
    declare_parameter("publish_confidence",  true);
    declare_parameter("disp_frame_id",       std::string("Passive/left_camera_link"));

    // Preview
    declare_parameter("preview_enabled",  true);
    declare_parameter("preview_width",    612);
    declare_parameter("preview_height",   512);
    declare_parameter("preview_quality",  50);
    declare_parameter("preview_fps",      10.0);   ///< Max framerate for preview publishing (e.g. 10 Hz)

    declare_parameter("namespace",        std::string("Passive"));
}

void PassiveStereoNode::init()
{
    const std::string ns = get_parameter("namespace").as_string();
    auto mk = [&](const std::string & topic) -> std::string {
        return "/" + ns + "/" + topic;
    };

    auto sensor_qos = rclcpp::SensorDataQoS();
    rclcpp::QoS best_effort_qos(2);
    best_effort_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // ── Stereo Rectifier ───────────────────────────────────────────────────────
    std::string calib_file = get_parameter("calibration_file").as_string();
    if (calib_file.empty()) {
        throw std::runtime_error(
            "passive_stereo_node: 'calibration_file' parameter must be set!");
    }
    calib_ = std::make_unique<StereoCalib>();
    calib_->load(calib_file);
    RCLCPP_INFO(get_logger(), "Calibration loaded: %dx%d, baseline=%.4f m",
        calib_->width(), calib_->height(), calib_->baseline());

    // ── Feature flags ──────────────────────────────────────────────────────────
    slam_enabled_    = get_parameter("slam_enabled").as_bool();
    disp_enabled_    = get_parameter("disparity_enabled").as_bool();
    prev_enabled_    = get_parameter("preview_enabled").as_bool();
    trigger_enabled_ = get_parameter("trigger_mode").as_bool();

    // ── SLAM Worker ────────────────────────────────────────────────────────────
    if (slam_enabled_) {
        auto pub_pose  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            mk("slam/pose_cov"), sensor_qos);
        auto pub_cloud = create_publisher<sensor_msgs::msg::PointCloud2>(
            mk("slam/pointcloud"), sensor_qos);
        auto pub_path  = create_publisher<nav_msgs::msg::Path>(
            mk("slam/path"), 10);

        SlamWorker::Config slam_cfg;
        slam_cfg.voc_file       = get_parameter("slam_voc_file").as_string();
        slam_cfg.settings_file  = get_parameter("slam_settings_file").as_string();
        slam_cfg.use_pangolin   = get_parameter("slam_use_pangolin").as_bool();
        slam_cfg.scale_factor   = get_parameter("slam_scale_factor").as_double();
        slam_cfg.frame_id       = get_parameter("frame_id").as_string();
        slam_cfg.parent_frame_id= get_parameter("parent_frame_id").as_string();
        slam_cfg.child_frame_id = get_parameter("child_frame_id").as_string();
        slam_cfg.enu_publish    = get_parameter("slam_enu_publish").as_bool();
        slam_cfg.tf_publish     = get_parameter("slam_tf_publish").as_bool();
        slam_cfg.cloud_pub_hz   = get_parameter("slam_cloud_hz").as_double();
        slam_cfg.clahe_clip     = get_parameter("clahe_clip_limit").as_double();
        slam_cfg.clahe_tiles    = get_parameter("clahe_grid_size").as_int();

        slam_worker_ = std::make_unique<SlamWorker>(
            this, pub_pose, pub_cloud, pub_path, slam_cfg);
        slam_worker_->start();
        RCLCPP_INFO(get_logger(), "SLAM worker started");

        reset_srv_ = create_service<std_srvs::srv::Trigger>(
            mk("slam/reset"),
            std::bind(&PassiveStereoNode::onSlamReset, this,
                      std::placeholders::_1, std::placeholders::_2));
    }

    // ── Disparity Worker ───────────────────────────────────────────────────────
    if (disp_enabled_) {
        auto pub_cloud = create_publisher<sensor_msgs::msg::PointCloud2>(
            mk("disparity/pointcloud"), sensor_qos);

        DisparityWorker::Config disp_cfg;
        disp_cfg.depth_mode          = get_parameter("depth_mode").as_string();
        disp_cfg.max_dist            = get_parameter("max_dist").as_double();
        disp_cfg.sampling_factor     = get_parameter("sampling_factor").as_double();
        disp_cfg.crop_factor         = get_parameter("crop_factor").as_double();
        disp_cfg.min_confidence      = get_parameter("min_confidence").as_double();
        disp_cfg.confidence_radius   = get_parameter("confidence_radius").as_int();
        disp_cfg.confidence_alpha    = get_parameter("confidence_alpha").as_double();
        disp_cfg.publish_confidence  = get_parameter("publish_confidence").as_bool();
        disp_cfg.frame_id            = get_parameter("disp_frame_id").as_string();
        disp_cfg.clahe_clip           = get_parameter("clahe_clip_limit").as_double();
        disp_cfg.clahe_tiles          = get_parameter("clahe_tile_size").as_int();
        disp_cfg.clahe_enabled        = get_parameter("enable_clahe").as_bool();

        disp_worker_ = std::make_unique<DisparityWorker>(pub_cloud, *calib_, disp_cfg);
        disp_worker_->start();
        RCLCPP_INFO(get_logger(), "Disparity worker started");
    }

    // ── Preview Worker ─────────────────────────────────────────────────────────
    if (prev_enabled_) {
        auto pub_left  = create_publisher<sensor_msgs::msg::CompressedImage>(
            mk("left/preview/image/compressed"), best_effort_qos);
        auto pub_right = create_publisher<sensor_msgs::msg::CompressedImage>(
            mk("right/preview/image/compressed"), best_effort_qos);

        PreviewWorker::Config prev_cfg;

        prev_cfg.preview_width  = get_parameter("preview_width").as_int();
        prev_cfg.preview_height = get_parameter("preview_height").as_int();
        prev_cfg.jpeg_quality   = get_parameter("preview_quality").as_int();
        prev_cfg.max_fps        = get_parameter("preview_fps").as_double();
        prev_cfg.clahe_clip     = get_parameter("clahe_clip_limit").as_double();
        prev_cfg.clahe_tiles    = get_parameter("clahe_grid_size").as_int();
        prev_cfg.clahe_enabled  = get_parameter("enable_clahe").as_bool();

        prev_worker_ = std::make_unique<PreviewWorker>(pub_left, pub_right, prev_cfg);
        prev_worker_->start();
        RCLCPP_INFO(get_logger(), "Preview worker started (%dx%d, q=%d, max_fps=%.1f)",
            prev_cfg.preview_width, prev_cfg.preview_height, prev_cfg.jpeg_quality, prev_cfg.max_fps);
    }

    // ── GPIO Trigger ───────────────────────────────────────────────────────────
    if (trigger_enabled_) {
        std::string chip = get_parameter("gpio_chip").as_string();
        int line         = get_parameter("gpio_line").as_int();
        double fps       = get_parameter("frame_rate").as_double();

        gpio_trigger_ = std::make_unique<GpioTrigger>(
            chip, static_cast<unsigned int>(line), fps);
        gpio_trigger_->start();
        RCLCPP_INFO(get_logger(), "GPIO PWM trigger started: chip=%s line=%d @%.1f Hz",
            chip.c_str(), line, fps);
    }

    // ── Preprocess Thread ──────────────────────────────────────────────────────
    preprocess_running_.store(true);
    preprocess_thread_ = std::thread(&PassiveStereoNode::preprocessThread, this);

    // ── Spinnaker Grabber ──────────────────────────────────────────────────────
    CameraConfig cam_cfg;
    cam_cfg.serial_left            = get_parameter("cam_left_serial").as_string();
    cam_cfg.serial_right           = get_parameter("cam_right_serial").as_string();
    cam_cfg.frame_rate             = get_parameter("frame_rate").as_double();
    cam_cfg.exposure_time_us       = get_parameter("exposure_time").as_double();
    cam_cfg.gain_db                = get_parameter("gain").as_double();
    cam_cfg.gain_auto              = get_parameter("gain_auto").as_bool();
    cam_cfg.balance_white_auto     = get_parameter("balance_white_auto").as_bool();
    cam_cfg.trigger_mode           = trigger_enabled_;
    cam_cfg.trigger_delay_us       = get_parameter("trigger_delay_us").as_int();
    cam_cfg.binning                = get_parameter("binning").as_int();
    cam_cfg.max_consecutive_errors = get_parameter("max_consec_errors").as_int();
    cam_cfg.frame_id_sync_tolerance = trigger_enabled_ ? 0 : 1;

    grabber_ = std::make_unique<SpinnakerGrabber>(cam_cfg);
    grabber_->setCallback([this](const RawFrame & l, const RawFrame & r) {
        this->onStereoFrame(l, r);
    });
    grabber_->start();

    RCLCPP_INFO(get_logger(),
        "Spinnaker cameras started (left=%s, right=%s, trigger=%s)",
        cam_cfg.serial_left.c_str(), cam_cfg.serial_right.c_str(),
        trigger_enabled_ ? "HW" : "continuous");
}

// Non-blocking: just push raw Bayer pair to preprocess queue and return immediately.
// This runs on the sync thread — must not block.
void PassiveStereoNode::onStereoFrame(
    const RawFrame & left_raw, const RawFrame & right_raw)
{
    preprocess_queue_.push({left_raw, right_raw});
}

void PassiveStereoNode::preprocessThread()
{
    while (preprocess_running_.load()) {
        RawPair pair;
        if (!preprocess_queue_.pop(pair)) continue;

        const RawFrame & left_raw  = pair.first;
        const RawFrame & right_raw = pair.second;

        cv::Mat left_rgb_raw, right_rgb_raw;
        cv::cvtColor(left_raw.image,  left_rgb_raw,  cv::COLOR_BayerRG2RGB);
        cv::cvtColor(right_raw.image, right_rgb_raw, cv::COLOR_BayerRG2RGB);

        // ── Build StereoFrame ─────────────────────────────────────────────────
        auto frame = std::make_shared<StereoFrame>();
        frame->left_raw          = left_raw.image;
        frame->right_raw         = right_raw.image;
        frame->left_rgb          = std::move(left_rgb_raw);
        frame->right_rgb         = std::move(right_rgb_raw);
        // Integer timestamp — no floating-point round-trip
        frame->timestamp_ns  = left_raw.timestamp_ns;
        frame->timestamp_sec = static_cast<double>(left_raw.timestamp_ns) * 1e-9;
        frame->stamp = rclcpp::Time(
            static_cast<int32_t>(left_raw.timestamp_ns / 1'000'000'000ULL),
            static_cast<uint32_t>(left_raw.timestamp_ns % 1'000'000'000ULL));
        frame->frame_id = left_raw.frame_id;

        // ── Dispatch to workers ───────────────────────────────────────────────
        if (slam_worker_)  slam_worker_->push(frame);
        if (disp_worker_)  disp_worker_->push(frame);
        if (prev_worker_)  prev_worker_->push(frame);
    }
}

void PassiveStereoNode::onSlamReset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    if (slam_worker_) slam_worker_->reset();
    res->success = true;
    res->message = "SLAM reset";
}

}  // namespace passive_stereo_capture
