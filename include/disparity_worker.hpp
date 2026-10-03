#pragma once

#include <thread>
#include <atomic>
#include <memory>
#include <vector>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cuda_runtime.h>
#include <retinify/retinify.hpp>
#include "clahe_processor.hpp"
#include "stereo_frame.hpp"
#include "bounded_queue.hpp"
#include "stereo_calib.hpp"

namespace passive_stereo_capture
{

/// Runs the Retinify GPU stereo depth pipeline in a dedicated thread.
/// Input: rectified RGB8 stereo pair (left/right — with CLAHE applied).
/// Output: dense coloured PointCloud2 published to ROS 2.
class DisparityWorker
{
public:
    struct Config {
        std::string depth_mode{"accurate"};   ///< "fast" | "balanced" | "accurate"
        double max_dist{15.0};                 ///< metres; 0 = no limit
        double sampling_factor{1.0};           ///< point cloud decimation (0-1]
        double crop_factor{1.0};               ///< central crop fraction (0-1]
        double min_confidence{0.35};
        int    confidence_radius{2};
        double confidence_alpha{2.0};
        bool   publish_confidence{true};
        std::string frame_id{"Passive/left_camera_link"};
        double clahe_clip{2.0};    ///< Grayscale CLAHE clip limit (applied after resize)
        int    clahe_tiles{8};     ///< Grayscale CLAHE tile grid size
        bool  clahe_enabled{false}; ///< Apply CLAHE to preview images (Bayer→BGR8)
    };

    using PointCloud2Pub = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;

    DisparityWorker(
        PointCloud2Pub pub_cloud,
        const StereoCalib & rectifier,   ///< Used for calibration params (P1/P2/baseline)
        const Config & cfg);

    ~DisparityWorker();

    void push(StereoFramePtr frame);
    void start();
    void stop();

private:
    /// Packed point layouts
    struct PointXYZRGB      { float x, y, z; uint32_t rgb; };
    struct PointXYZRGBConf  { float x, y, z; uint32_t rgb; float conf; };

    bool initPipeline(uint32_t width, uint32_t height);

    size_t compactCloud(
        const float * xyz, const uint8_t * img_rgb,
        uint32_t W, uint32_t H,
        int u0, int v0, int u1, int v1, int step,
        float max_dist_sq, int img_step,
        void * out_buf, bool with_conf,
        const float * disp, int disp_step,
        int conf_radius, float conf_alpha, float min_conf);

    void run();

    PointCloud2Pub pub_cloud_;
    Config         cfg_;
    StereoCalib    calib_;
    cv::Ptr<cv::CLAHE> clahe_{cv::createCLAHE(cfg_.clahe_clip, cv::Size(cfg_.clahe_tiles, cfg_.clahe_tiles))};

    retinify::Pipeline pipeline_;
    bool               pipeline_init_{false};
    uint32_t           pipeline_W_{0};  // cached pipeline width — detect resolution changes
    uint32_t           pipeline_H_{0};

    float * h_pinned_disp_{nullptr};
    size_t  pinned_disp_bytes_{0};
    float * h_pinned_xyz_{nullptr};
    size_t  pinned_xyz_bytes_{0};
    std::vector<float>   cpu_disp_buf_;

    // FIX #9: Pre-allocated point output buffer (reused every frame)
    std::vector<uint8_t> cpu_point_buf_;
    // FIX #9: Pre-allocated, reused PointCloud2 message data buffer
    std::vector<uint8_t> cloud_data_buf_;

    BoundedQueue<StereoFramePtr> queue_{1, /*drop_oldest=*/true};
    std::thread      thread_;
    std::atomic<bool> running_{false};
};

}  // namespace passive_stereo_capture
