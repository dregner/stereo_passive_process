#include "passive_stereo_node.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <iomanip>
#include <sstream>

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
    // 1. Stop grabber first
    if (grabber_) grabber_->stop();

    // 2. Drain and stop preprocess thread
    preprocess_running_.store(false);
    preprocess_queue_.shutdown();
    if (preprocess_thread_.joinable()) preprocess_thread_.join();

    // 3. Stop workers
    if (gpio_trigger_) gpio_trigger_->stop();
    if (slam_worker_)  slam_worker_->stop();
    if (disp_worker_)  disp_worker_->stop();
    if (prev_worker_)  prev_worker_->stop();
}

void PassiveStereoNode::declareParameters()
{
    // Camera
    declare_parameter("cam_left_serial",    "23267852");
    declare_parameter("cam_right_serial",   "23267854");
    declare_parameter("frame_rate",         30.0);
    declare_parameter("exposure_time",      33333.84);
    declare_parameter("gain",               0.0);
    declare_parameter("gain_auto",          false);
    declare_parameter("balance_white_auto", true);
    declare_parameter("binning",            1);
    declare_parameter("trigger_mode",       false);
    declare_parameter("trigger_delay_us",   29);
    declare_parameter("trigger_source", std::string("Line3"));
    declare_parameter("max_consec_errors",  10);
    declare_parameter("sync_tolerance_ms",  0.0);

    // GPIO
    declare_parameter("gpio_chip",         std::string("gpiochip0"));
    declare_parameter("gpio_line",         (int)85);   // PN.01 = GPIO27 = trigger line

    // Calibration
    declare_parameter("calibration_file",  std::string(""));

    // CLAHE
    declare_parameter("clahe_clip_limit",  2.0);
    declare_parameter("clahe_tile_size",   8);
    declare_parameter("enable_clahe",       false);

    // SLAM
    declare_parameter("slam_enabled",       true);
    declare_parameter("slam_voc_file",       std::string(""));
    declare_parameter("slam_settings_file",  std::string(""));
    declare_parameter("slam_use_pangolin",   false);
    declare_parameter("slam_scale_factor",   1.0);
    declare_parameter("slam_enu_publish",    true);
    declare_parameter("slam_tf_publish",     false);
    declare_parameter("slam_cloud_hz",       2.0);
    declare_parameter("frame_id",            std::string("map"));
    declare_parameter("parent_frame_id",     std::string("base_link"));
    declare_parameter("child_frame_id",      std::string("Passive/left_camera_link"));

    // Disparity
    declare_parameter("disparity_backend", std::string("retinify"));
    declare_parameter("stereo_min_disparity", 0);
    declare_parameter("stereo_num_disparities", 128);
    declare_parameter("stereo_block_size", 9);
    declare_parameter("stereo_uniqueness_ratio", 10);
    declare_parameter("stereo_speckle_window_size", 100);
    declare_parameter("stereo_speckle_range", 2);
    declare_parameter("stereo_disp12_max_diff", 1);
    declare_parameter("stereo_pre_filter_cap", 31);
    declare_parameter("stereo_texture_threshold", 10);
    declare_parameter("disparity_enabled",   true);
    declare_parameter("disp_publish_cloud", false);
    declare_parameter("depth_mode",          std::string("accurate"));
    declare_parameter("max_dist",            15.0);
    declare_parameter("sampling_factor",     1.0);
    declare_parameter("crop_factor",         1.0);
    declare_parameter("min_confidence",      0.35);
    declare_parameter("confidence_radius",   2);
    declare_parameter("confidence_alpha",    2.0);
    declare_parameter("publish_confidence",  true);
    declare_parameter("disp_frame_id",       std::string("Passive/left_camera_link"));
    declare_parameter("depth_process_hz", 0.0);
    declare_parameter("disparity_rectify_on_cpu", false);
    declare_parameter("disparity_trace_path", std::string(""));
    declare_parameter("opencv_threads", 0);
    declare_parameter("disp_cloud_hz",       15.0);
    declare_parameter("depth_width",         0);
    declare_parameter("depth_height",        0);
    declare_parameter("disp_image_hz",       10.0);

    // Preview
    declare_parameter("preview_enabled",  true);
    declare_parameter("preview_width",    612);
    declare_parameter("preview_height",   512);
    declare_parameter("preview_quality",  50);
    declare_parameter("preview_fps",      10.0);

    declare_parameter("namespace",        std::string("Passive"));
}

void PassiveStereoNode::init()
{
    const int cv_threads = get_parameter("opencv_threads").as_int();
    if (cv_threads > 0) cv::setNumThreads(cv_threads);
    const std::string ns = get_parameter("namespace").as_string();
    auto mk = [&](const std::string & topic) -> std::string {
        return "/" + ns + "/" + topic;
    };

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    rclcpp::QoS best_effort_qos(2);
    best_effort_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    // ── Calibration ───────────────────────────────────────────────────────────
    std::string calib_file = get_parameter("calibration_file").as_string();
    if (calib_file.empty()) {
        throw std::runtime_error("passive_stereo_node: 'calibration_file' parameter must be set!");
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

    int clahe_tiles_val = get_parameter("clahe_tile_size").as_int();
    double clahe_clip_val = get_parameter("clahe_clip_limit").as_double();
    bool enable_clahe_val = get_parameter("enable_clahe").as_bool();

    // ── SLAM Worker ────────────────────────────────────────────────────────────
    if (slam_enabled_) {
        auto pub_pose  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            mk("slam/pose_cov"), sensor_qos);
        auto pub_cloud = create_publisher<sensor_msgs::msg::PointCloud2>(
            mk("slam/pointcloud"), sensor_qos);
        auto pub_path  = create_publisher<nav_msgs::msg::Path>(
            mk("slam/path"), 10);

        SlamWorker::Config slam_cfg;
        slam_cfg.voc_file        = get_parameter("slam_voc_file").as_string();
        slam_cfg.settings_file   = get_parameter("slam_settings_file").as_string();
        slam_cfg.use_pangolin    = get_parameter("slam_use_pangolin").as_bool();
        slam_cfg.scale_factor    = get_parameter("slam_scale_factor").as_double();
        slam_cfg.frame_id        = get_parameter("frame_id").as_string();
        slam_cfg.parent_frame_id = get_parameter("parent_frame_id").as_string();
        slam_cfg.child_frame_id  = get_parameter("child_frame_id").as_string();
        slam_cfg.enu_publish     = get_parameter("slam_enu_publish").as_bool();
        slam_cfg.tf_publish      = get_parameter("slam_tf_publish").as_bool();
        slam_cfg.cloud_pub_hz    = get_parameter("slam_cloud_hz").as_double();
        slam_cfg.clahe_clip      = clahe_clip_val;
        slam_cfg.clahe_tiles     = clahe_tiles_val;
        slam_cfg.clahe_enabled   = enable_clahe_val;

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
        DisparityWorker::PointCloud2Pub pub_cloud;
        if (get_parameter("disp_publish_cloud").as_bool())
            pub_cloud = create_publisher<sensor_msgs::msg::PointCloud2>(
                mk("disparity/pointcloud"), sensor_qos);
        auto pub_disp_img = create_publisher<sensor_msgs::msg::CompressedImage>(
            mk("disparity/image/compressed"), best_effort_qos);

        DisparityWorker::Config disp_cfg;
        disp_cfg.backend = get_parameter("disparity_backend").as_string();
        disp_cfg.stereo.min_disparity = get_parameter("stereo_min_disparity").as_int();
        disp_cfg.stereo.num_disparities = get_parameter("stereo_num_disparities").as_int();
        disp_cfg.stereo.block_size = get_parameter("stereo_block_size").as_int();
        disp_cfg.stereo.uniqueness_ratio = get_parameter("stereo_uniqueness_ratio").as_int();
        disp_cfg.stereo.speckle_window_size = get_parameter("stereo_speckle_window_size").as_int();
        disp_cfg.stereo.speckle_range = get_parameter("stereo_speckle_range").as_int();
        disp_cfg.stereo.disp12_max_diff = get_parameter("stereo_disp12_max_diff").as_int();
        disp_cfg.stereo.pre_filter_cap = get_parameter("stereo_pre_filter_cap").as_int();
        disp_cfg.stereo.texture_threshold = get_parameter("stereo_texture_threshold").as_int();
        disp_cfg.depth_mode          = get_parameter("depth_mode").as_string();
        disp_cfg.max_dist            = get_parameter("max_dist").as_double();
        disp_cfg.sampling_factor     = get_parameter("sampling_factor").as_double();
        disp_cfg.crop_factor         = get_parameter("crop_factor").as_double();
        disp_cfg.min_confidence      = get_parameter("min_confidence").as_double();
        disp_cfg.confidence_radius   = get_parameter("confidence_radius").as_int();
        disp_cfg.confidence_alpha    = get_parameter("confidence_alpha").as_double();
        disp_cfg.publish_confidence  = get_parameter("publish_confidence").as_bool();
        disp_cfg.frame_id            = get_parameter("disp_frame_id").as_string();
        disp_cfg.clahe_clip          = clahe_clip_val;
        disp_cfg.clahe_tiles         = clahe_tiles_val;
        disp_cfg.clahe_enabled       = enable_clahe_val;
        disp_cfg.width               = get_parameter("depth_width").as_int();
        disp_cfg.height              = get_parameter("depth_height").as_int();
        if (disp_cfg.width < 0 || disp_cfg.height < 0 ||
            ((disp_cfg.width == 0) != (disp_cfg.height == 0))) {
            throw std::invalid_argument("depth_width/depth_height must both be zero or both positive");
        }
        disp_cfg.process_hz = get_parameter("depth_process_hz").as_double();
        disp_cfg.rectify_on_cpu = get_parameter("disparity_rectify_on_cpu").as_bool();
        disp_cfg.trace_path = get_parameter("disparity_trace_path").as_string();
        if (!std::isfinite(disp_cfg.process_hz) || disp_cfg.process_hz < 0)
            throw std::invalid_argument("Depth processing rate must be finite and nonnegative");
        disp_cfg.cloud_hz            = get_parameter("disp_cloud_hz").as_double();
        disp_cfg.image_hz            = get_parameter("disp_image_hz").as_double();

        disp_worker_ = std::make_unique<DisparityWorker>(pub_cloud, pub_disp_img, *calib_, disp_cfg);
        disp_worker_->start();
        RCLCPP_INFO(get_logger(), "Disparity worker started (%s)", disp_cfg.backend.c_str());
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
        prev_cfg.clahe_clip     = clahe_clip_val;
        prev_cfg.clahe_tiles    = clahe_tiles_val;
        prev_cfg.clahe_enabled  = enable_clahe_val;

        prev_worker_ = std::make_unique<PreviewWorker>(pub_left, pub_right, prev_cfg);
        prev_worker_->start();
        RCLCPP_INFO(get_logger(), "Preview worker started (%dx%d, q=%d, max_fps=%.1f)",
            prev_cfg.preview_width, prev_cfg.preview_height, prev_cfg.jpeg_quality, prev_cfg.max_fps);
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
    cam_cfg.trigger_source = get_parameter("trigger_source").as_string();
    cam_cfg.trigger_delay_us       = get_parameter("trigger_delay_us").as_int();
    cam_cfg.binning                = get_parameter("binning").as_int();
    cam_cfg.max_consecutive_errors = get_parameter("max_consec_errors").as_int();
    cam_cfg.sync_tolerance_ms      = get_parameter("sync_tolerance_ms").as_double();

    grabber_ = std::make_unique<SpinnakerGrabber>(cam_cfg);
    grabber_->setCallback([this](const RawFrame & l, const RawFrame & r) {
        this->onStereoFrame(l, r);
    });
    grabber_->start();

    // ── GPIO Trigger (started AFTER cameras are fully armed!) ─────────────────
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

    report_timer_ = create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&PassiveStereoNode::reportTimerCallback, this));

    RCLCPP_INFO(get_logger(),
        "Spinnaker cameras started (left=%s, right=%s, trigger=%s, sync_tol=%.1f ms)",
        grabber_->serialLeft().c_str(), grabber_->serialRight().c_str(),
        trigger_enabled_ ? "HW" : "continuous",
        grabber_->syncToleranceMs());
}

void PassiveStereoNode::reportTimerCallback()
{
    if (!grabber_) return;
    if (trigger_enabled_ && gpio_trigger_ && !gpio_trigger_->isRunning()) {
        RCLCPP_ERROR(get_logger(), "GPIO trigger failed; stopping acquisition instead of waiting for missing pulses");
        rclcpp::shutdown();
        return;
    }
    auto s = grabber_->stats();

    std::stringstream ss;
    ss << "[Pipeline Status] Grabbed: L=" << s.left_frames << " R=" << s.right_frames
       << " | Pairs=" << s.pairs << " | Dropped=" << s.dropped_unmatched
       << " | Incomplete=" << s.incomplete;

    if (disp_worker_) {
        ss << " | DispFrames=" << disp_worker_->processedFrames()
           << " clouds=" << disp_worker_->publishedClouds()
           << " empty_clouds=" << disp_worker_->emptyClouds()
           << " dropped=" << disp_worker_->droppedFrames()
           << " process_ms=" << disp_worker_->processingMs()
           << " receipt_age_ms=" << disp_worker_->receiptAgeMs();
    }
    if (slam_worker_) {
        ss << " | SlamFrames=" << slam_worker_->processedFrames()
           << " (State: " << slam_worker_->lastTrackingState() << ")"
           << " dropped=" << slam_worker_->droppedFrames()
           << " process_ms=" << slam_worker_->processingMs()
           << " receipt_age_ms=" << slam_worker_->receiptAgeMs();
    }

    ss << " | ColorDropped=" << preprocess_queue_.dropped();
    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
}

void PassiveStereoNode::onStereoFrame(
    const RawFrame & left_raw, const RawFrame & right_raw)
{
    auto frame = std::make_shared<StereoFrame>();
    frame->left_raw = left_raw.image;
    frame->right_raw = right_raw.image;
    frame->received_at = std::min(left_raw.received_at, right_raw.received_at);
    // Receipt timestamp is a fallback until camera-to-host clock mapping is available.
    frame->stamp = rclcpp::Time(static_cast<int64_t>(left_raw.host_ns), RCL_ROS_TIME);
    frame->timestamp_ns = left_raw.timestamp_ns;
    frame->timestamp_sec = static_cast<double>(left_raw.timestamp_ns) * 1e-9;
    frame->frame_id = left_raw.frame_id;

    // SLAM never waits for color conversion. Consumers do not modify this frame.
    if (slam_worker_) slam_worker_->push(frame);
    if (disp_worker_ || prev_worker_) preprocess_queue_.push(std::move(frame));
}

void PassiveStereoNode::preprocessThread()
{
    while (preprocess_running_.load()) {
        StereoFramePtr raw_frame;
        if (!preprocess_queue_.pop(raw_frame)) continue;
        // Separate frame metadata prevents races with the already running SLAM worker.
        auto color_frame = std::make_shared<StereoFrame>(*raw_frame);
        cv::cvtColor(raw_frame->left_raw, color_frame->left_rgb, cv::COLOR_BayerRG2RGB);
        cv::cvtColor(raw_frame->right_raw, color_frame->right_rgb, cv::COLOR_BayerRG2RGB);
        if (disp_worker_) disp_worker_->push(color_frame);
        if (prev_worker_) prev_worker_->push(color_frame);
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
