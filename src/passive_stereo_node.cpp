#include "passive_stereo_node.hpp"

#include <chrono>
#include <stdexcept>

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
    // Stop in reverse-init order
    if (gpio_trigger_) gpio_trigger_->stop();
    if (grabber_)      grabber_->stop();
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

    // GPIO (for hardware trigger generation)
    declare_parameter("gpio_chip",         std::string("gpiochip0"));
    declare_parameter("gpio_line",         (int)106);   // example Jetson Orin pin 16

    // Calibration
    declare_parameter("calibration_file",  std::string(""));

    // CLAHE
    declare_parameter("clahe_clip_limit",  2.0);
    declare_parameter("clahe_tile_size",   8);

    // SLAM
    declare_parameter("slam_enabled",       true);
    declare_parameter("slam_voc_file",       std::string(""));
    declare_parameter("slam_settings_file",  std::string(""));
    declare_parameter("slam_use_pangolin",   false);
    declare_parameter("slam_max_width",      800);
    declare_parameter("slam_max_height",     600);
    declare_parameter("slam_enu_publish",    true);
    declare_parameter("slam_tf_publish",     false);
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

    // Namespace prefix (for topics)
    declare_parameter("namespace",        std::string("Passive"));
}

void PassiveStereoNode::init()
{
    const std::string ns = get_parameter("namespace").as_string();
    auto mk = [&](const std::string & topic) -> std::string {
        return "/" + ns + "/" + topic;
    };

    // ── QoS ────────────────────────────────────────────────────────────────────
    auto sensor_qos = rclcpp::SensorDataQoS();
    rclcpp::QoS best_effort_qos(2);
    best_effort_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // ── CLAHE ──────────────────────────────────────────────────────────────────
    double clip  = get_parameter("clahe_clip_limit").as_double();
    int    tiles = get_parameter("clahe_tile_size").as_int();
    clahe_ = cv::createCLAHE(clip, cv::Size(tiles, tiles));

    // ── Stereo Rectifier ───────────────────────────────────────────────────────
    std::string calib_file = get_parameter("calibration_file").as_string();
    if (calib_file.empty()) {
        throw std::runtime_error(
            "passive_stereo_node: 'calibration_file' parameter must be set!");
    }
    rectifier_ = std::make_unique<StereoRectifier>();
    rectifier_->load(calib_file);
    RCLCPP_INFO(get_logger(), "Calibration loaded: %dx%d, baseline=%.4f m",
        rectifier_->width(), rectifier_->height(), rectifier_->baseline());

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
        slam_cfg.max_width      = get_parameter("slam_max_width").as_int();
        slam_cfg.max_height     = get_parameter("slam_max_height").as_int();
        slam_cfg.frame_id       = get_parameter("frame_id").as_string();
        slam_cfg.parent_frame_id= get_parameter("parent_frame_id").as_string();
        slam_cfg.child_frame_id = get_parameter("child_frame_id").as_string();
        slam_cfg.enu_publish    = get_parameter("slam_enu_publish").as_bool();
        slam_cfg.tf_publish     = get_parameter("slam_tf_publish").as_bool();

        slam_worker_ = std::make_unique<SlamWorker>(
            this, pub_pose, pub_cloud, pub_path, slam_cfg);
        slam_worker_->start();
        RCLCPP_INFO(get_logger(), "SLAM worker started");

        // Service to reset SLAM
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

        disp_worker_ = std::make_unique<DisparityWorker>(pub_cloud, *rectifier_, disp_cfg);
        disp_worker_->start();
        RCLCPP_INFO(get_logger(), "Disparity worker started");
    }

    // ── Preview Worker ─────────────────────────────────────────────────────────
    if (prev_enabled_) {
        auto pub_left  = create_publisher<sensor_msgs::msg::CompressedImage>(
            mk("left/preview/image/compressed"), best_effort_qos);
        auto pub_right = create_publisher<sensor_msgs::msg::CompressedImage>(
            mk("right/preview/image/compressed"), best_effort_qos);

        int pw = get_parameter("preview_width").as_int();
        int ph = get_parameter("preview_height").as_int();
        int pq = get_parameter("preview_quality").as_int();

        prev_worker_ = std::make_unique<PreviewWorker>(pub_left, pub_right, pw, ph, pq);
        prev_worker_->start();
        RCLCPP_INFO(get_logger(), "Preview worker started (%dx%d, q=%d)", pw, ph, pq);
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

    // ── Spinnaker Grabber ──────────────────────────────────────────────────────
    CameraConfig cam_cfg;
    cam_cfg.serial_left       = get_parameter("cam_left_serial").as_string();
    cam_cfg.serial_right      = get_parameter("cam_right_serial").as_string();
    cam_cfg.frame_rate        = get_parameter("frame_rate").as_double();
    cam_cfg.exposure_time_us  = get_parameter("exposure_time").as_double();
    cam_cfg.gain_db           = get_parameter("gain").as_double();
    cam_cfg.gain_auto         = get_parameter("gain_auto").as_bool();
    cam_cfg.balance_white_auto= get_parameter("balance_white_auto").as_bool();
    cam_cfg.trigger_mode      = trigger_enabled_;
    cam_cfg.trigger_delay_us  = get_parameter("trigger_delay_us").as_int();
    cam_cfg.binning           = get_parameter("binning").as_int();
    // In HW trigger mode allow exact FrameID match; in continuous allow ±1
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

void PassiveStereoNode::onStereoFrame(
    const RawFrame & left_raw, const RawFrame & right_raw)
{
    // ── 1. Rectify ────────────────────────────────────────────────────────────
    cv::Mat left_rect, right_rect;
    rectifier_->rectify(left_raw.image, right_raw.image, left_rect, right_rect);

    // ── 2. CLAHE (Lab L-channel) ──────────────────────────────────────────────
    left_rect  = applyClaheRGB(left_rect,  clahe_);
    right_rect = applyClaheRGB(right_rect, clahe_);

    // ── 3. Build StereoFrame ──────────────────────────────────────────────────
    // Use left camera timestamp; in HW trigger mode both are (effectively) identical
    auto frame = std::make_shared<StereoFrame>();
    frame->left          = std::move(left_rect);
    frame->right         = std::move(right_rect);
    frame->timestamp_sec = left_raw.timestamp_sec;
    frame->frame_id      = left_raw.frame_id;
    // Build rclcpp::Time from seconds (Spinnaker clock)
    uint64_t ns = static_cast<uint64_t>(left_raw.timestamp_sec * 1e9);
    frame->stamp = rclcpp::Time(static_cast<int32_t>(ns / 1'000'000'000ULL),
                                 static_cast<uint32_t>(ns % 1'000'000'000ULL));

    // ── 4. Dispatch to workers (non-blocking push) ────────────────────────────
    if (slam_worker_)  slam_worker_->push(frame);
    if (disp_worker_)  disp_worker_->push(frame);
    if (prev_worker_)  prev_worker_->push(frame);
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
