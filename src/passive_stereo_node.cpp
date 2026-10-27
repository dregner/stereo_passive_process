#include "passive_stereo_node.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <iomanip>
#include <sstream>

#include <opencv2/imgproc.hpp>
#include "cuda_color_converter.hpp"
#include "color_correction.hpp"
#include "spinnaker_color_correction.hpp"

namespace passive_stereo_capture
{

PassiveStereoNode::PassiveStereoNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("passive_stereo_node", options)
{
    declareParameters();
    try { init(); }
    catch (...) {
        stopPipeline();
        throw;
    }
}

PassiveStereoNode::~PassiveStereoNode()
{
    stopPipeline();
}

void PassiveStereoNode::stopPipeline()
{
    // Stop trigger outputs before tearing down acquisition.
    if (pwm_trigger_) pwm_trigger_->stop();
    if (gpio_trigger_) gpio_trigger_->stop();
    if (grabber_) grabber_->stop();

    // 2. Drain and stop preprocess thread
    preprocess_running_.store(false);
    preprocess_queue_.shutdown();
    if (preprocess_thread_.joinable()) preprocess_thread_.join();

    // 3. Stop workers
    if (slam_worker_)  slam_worker_->stop();
    if (mavlink_worker_) mavlink_worker_->stop();
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
    declare_parameter("color_correction_enabled", false);
    declare_parameter("color_correction_backend", std::string("spinnaker"));
    declare_parameter("color_ccm_left", std::vector<double>{1,0,0,0,1,0,0,0,1});
    declare_parameter("color_ccm_right", std::vector<double>{1,0,0,0,1,0,0,0,1});
    declare_parameter("binning",            1);
    declare_parameter("trigger_mode",       false);
    declare_parameter("trigger_delay_us",   29);
    declare_parameter("trigger_source", std::string("Line3"));
    declare_parameter("trigger_activation", std::string("RisingEdge"));
    declare_parameter("trigger_generator", std::string("gpio")); // gpio | pwm | external
    declare_parameter("pwm_chip", std::string("/sys/class/pwm/pwmchip2"));
    declare_parameter("pwm_channel", 0);
    declare_parameter("pwm_duty_cycle_percent", 50.0);
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
    declare_parameter("stereo_cuda_p1", 10);
    declare_parameter("stereo_cuda_p2", 120);
    declare_parameter("opencv_reproject_on_gpu", false);
    declare_parameter("color_conversion_backend", std::string("cpu"));
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
    declare_parameter("depth_process_hz", 10.0);
    declare_parameter("disparity_nice", 5);
    declare_parameter("slam_trace_path", std::string(""));
    declare_parameter("disparity_rectify_on_cpu", false);
    declare_parameter("disparity_trace_path", std::string(""));
    declare_parameter("opencv_threads", 0);
    declare_parameter("disp_cloud_hz",       15.0);
    declare_parameter("depth_width",         0);
    declare_parameter("depth_height",        0);
    declare_parameter("disp_image_hz",       10.0);

    // MAVLink: opening a connection is opt-in, independent of preview/depth.
    declare_parameter("mavlink_enabled", false);
    declare_parameter("mavlink_bridge_url", std::string("udp://:14555@127.0.0.1:14550"));
    declare_parameter("mavlink_message", std::string("estimate"));
    declare_parameter("mavlink_system_id", 1);
    declare_parameter("mavlink_component_id", 197);
    declare_parameter("mavlink_max_gap_sec", 0.5);
    declare_parameter("mavlink_map_yaw_deg", 0.0);
    declare_parameter("mavlink_position_variance", 0.05);
    declare_parameter("mavlink_angle_variance", 0.1);
    declare_parameter("mavlink_confidence", 100.0);

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
    color_correction_enabled_ = get_parameter("color_correction_enabled").as_bool();
    if (color_correction_enabled_) {
        const auto backend = get_parameter("color_correction_backend").as_string();
        if (backend != "spinnaker" && backend != "opencv")
            throw std::invalid_argument("color_correction_backend must be spinnaker or opencv");
        color_ccm_left_ = colorCorrectionMatrix(get_parameter("color_ccm_left").as_double_array());
        color_ccm_right_ = colorCorrectionMatrix(get_parameter("color_ccm_right").as_double_array());
        RCLCPP_INFO(get_logger(), "Host RGB color correction enabled: %s (separate left/right CCMs)", backend.c_str());
    }

    const auto trigger_generator = get_parameter("trigger_generator").as_string();
    if (trigger_generator != "gpio" && trigger_generator != "pwm" && trigger_generator != "external")
        throw std::invalid_argument("trigger_generator must be gpio, pwm or external");
    if (get_parameter("trigger_mode").as_bool() && trigger_generator == "pwm") {
        const auto channel = get_parameter("pwm_channel").as_int();
        if (channel < 0 || channel > 2147483647)
            throw std::invalid_argument("pwm_channel must be a nonnegative 32-bit integer");
        pwm_trigger_ = std::make_unique<PwmTrigger>(
            get_parameter("pwm_chip").as_string(), static_cast<int>(channel),
            get_parameter("frame_rate").as_double(),
            get_parameter("pwm_duty_cycle_percent").as_double());
    }
    const auto color_backend = get_parameter("color_conversion_backend").as_string();
    if (color_backend != "cpu" && color_backend != "cuda")
        throw std::invalid_argument("color_conversion_backend must be cpu or cuda");
#ifndef PASSIVE_HAVE_OPENCV_CUDA
    if (color_backend == "cuda" || get_parameter("opencv_reproject_on_gpu").as_bool())
        throw std::invalid_argument("CUDA processing requires an OpenCV build with CUDA modules");
#endif
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

    if (get_parameter("mavlink_enabled").as_bool()) {
        if (!slam_enabled_ || !get_parameter("slam_enu_publish").as_bool())
            throw std::invalid_argument("MAVLink requires slam_enabled and slam_enu_publish");
        MavlinkWorker::Config cfg;
        cfg.bridge_url = get_parameter("mavlink_bridge_url").as_string();
        cfg.message = get_parameter("mavlink_message").as_string();
        cfg.system_id = get_parameter("mavlink_system_id").as_int();
        cfg.component_id = get_parameter("mavlink_component_id").as_int();
        cfg.max_gap_sec = get_parameter("mavlink_max_gap_sec").as_double();
        cfg.map_yaw_deg = get_parameter("mavlink_map_yaw_deg").as_double();
        cfg.position_variance = get_parameter("mavlink_position_variance").as_double();
        cfg.angle_variance = get_parameter("mavlink_angle_variance").as_double();
        cfg.confidence = get_parameter("mavlink_confidence").as_double();
        mavlink_worker_ = std::make_unique<MavlinkWorker>(cfg,
            [this] { return static_cast<uint64_t>(now().nanoseconds() / 1000); },
            [this](const std::string & error) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Direct MAVLink send failed: %s", error.c_str());
            });
        RCLCPP_INFO(get_logger(), "MAVLink vision waits for camera-to-base TF before sending");
    }

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
        slam_cfg.trace_path      = get_parameter("slam_trace_path").as_string();

        if (mavlink_worker_) {
            slam_cfg.vision_sink = [this](VisionSample sample) {
                mavlink_worker_->push(std::move(sample));
            };
        }
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
        disp_cfg.stereo.cuda_p1 = get_parameter("stereo_cuda_p1").as_int();
        disp_cfg.stereo.cuda_p2 = get_parameter("stereo_cuda_p2").as_int();
        disp_cfg.reproject_on_gpu = get_parameter("opencv_reproject_on_gpu").as_bool();
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
        disp_cfg.nice = get_parameter("disparity_nice").as_int();
        if (disp_cfg.nice < 0 || disp_cfg.nice > 19)
            throw std::invalid_argument("disparity_nice must be in [0, 19]");
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
    cam_cfg.host_color_correction  = color_correction_enabled_;
    cam_cfg.balance_white_auto     = get_parameter("balance_white_auto").as_bool();
    cam_cfg.trigger_mode           = trigger_enabled_;
    cam_cfg.trigger_source = get_parameter("trigger_source").as_string();
    cam_cfg.trigger_activation = get_parameter("trigger_activation").as_string();
    cam_cfg.trigger_delay_us       = get_parameter("trigger_delay_us").as_int();
    cam_cfg.binning                = get_parameter("binning").as_int();
    cam_cfg.max_consecutive_errors = get_parameter("max_consec_errors").as_int();
    cam_cfg.sync_tolerance_ms      = get_parameter("sync_tolerance_ms").as_double();

    grabber_ = std::make_unique<SpinnakerGrabber>(cam_cfg);
    grabber_->setCallback([this](const RawFrame & l, const RawFrame & r) {
        this->onStereoFrame(l, r);
    });
    grabber_->start();

    // Trigger outputs start only AFTER both cameras are armed.
    if (pwm_trigger_) {
        pwm_trigger_->start();
        RCLCPP_INFO(get_logger(), "Hardware PWM trigger started: %s/pwm%ld @%.3f Hz, %.3f%% duty",
            get_parameter("pwm_chip").as_string().c_str(),
            static_cast<long>(get_parameter("pwm_channel").as_int()),
            get_parameter("frame_rate").as_double(),
            get_parameter("pwm_duty_cycle_percent").as_double());
    }
    if (trigger_enabled_ && trigger_generator == "gpio") {
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
    RCLCPP_DEBUG(get_logger(), "%s", ss.str().c_str());
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
    // The stream and buffers belong only to the color worker. SLAM still receives
    // raw Bayer frames first and never waits for GPU color conversion.
    CudaColorConverter cuda_color;
    const bool use_cuda = get_parameter("color_conversion_backend").as_string() == "cuda";
    const bool sdk_ccm = color_correction_enabled_ &&
        get_parameter("color_correction_backend").as_string() == "spinnaker";
    Spinnaker::CCMSettings left_settings, right_settings;
    try {
        if (sdk_ccm) {
            left_settings = spinnakerCcmSettings(get_parameter("color_ccm_left").as_double_array());
            right_settings = spinnakerCcmSettings(get_parameter("color_ccm_right").as_double_array());
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Spinnaker CCM initialization failed: %s", e.what());
        rclcpp::shutdown();
        return;
    }
    while (preprocess_running_.load()) {
        StereoFramePtr raw_frame;
        if (!preprocess_queue_.pop(raw_frame)) continue;
        // Separate frame metadata prevents races with the already running SLAM worker.
        auto color_frame = std::make_shared<StereoFrame>(*raw_frame);
        if (use_cuda) {
            try { cuda_color.convertPair(raw_frame->left_raw, raw_frame->right_raw,
                color_frame->left_rgb, color_frame->right_rgb); }
            catch (const std::exception & e) {
                RCLCPP_ERROR(get_logger(), "CUDA color conversion failed: %s", e.what());
                rclcpp::shutdown();
                return;
            }
        } else {
            cv::cvtColor(raw_frame->left_raw, color_frame->left_rgb, cv::COLOR_BayerRG2RGB);
            cv::cvtColor(raw_frame->right_raw, color_frame->right_rgb, cv::COLOR_BayerRG2RGB);
        }
        if (color_correction_enabled_) {
            try {
                if (sdk_ccm) {
                    correctRgbSpinnaker(color_frame->left_rgb, left_settings);
                    correctRgbSpinnaker(color_frame->right_rgb, right_settings);
                } else {
                    correctRgb(color_frame->left_rgb, color_ccm_left_);
                    correctRgb(color_frame->right_rgb, color_ccm_right_);
                }
            } catch (const std::exception & e) {
                RCLCPP_ERROR(get_logger(), "Color correction failed: %s", e.what());
                rclcpp::shutdown();
                return;
            }
        }
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
