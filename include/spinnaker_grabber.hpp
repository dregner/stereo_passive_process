#pragma once

#include <string>
#include <functional>
#include <atomic>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <memory>
#include <cstdint>

#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <opencv2/core.hpp>

namespace passive_stereo_capture
{

/// Configuration for camera acquisition
struct CameraConfig {
    std::string serial_left;
    std::string serial_right;
    double      frame_rate{30.0};
    double      exposure_time_us{33333.84};  ///< microseconds
    double      gain_db{0.0};               ///< dB; applied only if gain_auto=Off
    bool        gain_auto{false};
    bool        balance_white_auto{true};
    bool        trigger_mode{false};        ///< true = hardware trigger on Line3
    int         trigger_delay_us{29};       ///< microseconds
    int         binning{1};                 ///< 1 = full resolution
    int         acquire_timeout_ms{2000};
    int         frame_id_sync_tolerance{0}; ///< max FrameID difference for soft sync (0 = exact)
    int         max_consecutive_errors{10}; ///< consecutive grab errors before shutdown
};

/// A raw frame straight from the sensor — BayerRG8, no conversion.
/// Downstream workers choose their own conversion (Gray for SLAM, RGB for Retinify/Preview).
struct RawFrame {
    cv::Mat    image;           ///< CV_8UC1, BayerRG8 pattern (unconverted sensor data)
    uint64_t   frame_id{0};
    uint64_t   timestamp_ns{0}; ///< Spinnaker chunk timestamp in nanoseconds
};

/// Callback type: called on each synchronized stereo pair
using StereoCallback = std::function<void(const RawFrame & left, const RawFrame & right)>;

/// Acquires synchronized stereo frames from two BFS Spinnaker cameras.
/// Delivers raw BayerRG8 frames — all pixel format conversion is deferred to consumers.
/// In hardware trigger mode, both cameras fire on the same GPIO pulse;
/// frames are matched by FrameID from chunk data.
/// In continuous mode, frames are matched by closest FrameID.
class SpinnakerGrabber
{
public:
    explicit SpinnakerGrabber(const CameraConfig & cfg);
    ~SpinnakerGrabber();

    /// Set callback invoked for each synchronized stereo pair.
    void setCallback(StereoCallback cb) { callback_ = std::move(cb); }

    /// Open cameras, configure, and start acquisition threads.
    void start();

    /// Stop acquisition and release cameras.
    void stop();

    bool isRunning() const { return running_.load(); }

private:
    void configureCamera(Spinnaker::CameraPtr cam, bool is_left);
    void grabThread(Spinnaker::CameraPtr cam, bool is_left);
    void syncThread();

    CameraConfig   cfg_;
    StereoCallback callback_;

    Spinnaker::SystemPtr    system_;
    Spinnaker::CameraPtr    cam_left_;
    Spinnaker::CameraPtr    cam_right_;

    // Per-camera raw frame queues for synchronization
    std::queue<RawFrame>    left_queue_;
    std::queue<RawFrame>    right_queue_;
    std::mutex              left_mtx_;
    std::mutex              right_mtx_;
    std::condition_variable left_cv_;
    std::condition_variable right_cv_;

    std::thread left_grab_thread_;
    std::thread right_grab_thread_;
    std::thread sync_thread_;

    std::atomic<bool> running_{false};

    static constexpr std::size_t kQueueDepth = 8;
};

}  // namespace passive_stereo_capture
