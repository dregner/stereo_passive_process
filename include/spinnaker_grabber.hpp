#pragma once

#include <string>
#include <functional>
#include <atomic>
#include <thread>
#include <mutex>
#include <deque>
#include <condition_variable>
#include <memory>
#include <cstdint>
#include <chrono>

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
    double      exposure_time_us{33333.84};  ///< microseconds (clamped to the frame period)
    double      gain_db{0.0};               ///< dB; applied only if gain_auto=Off
    bool        gain_auto{false};
    bool        balance_white_auto{true};
    bool        trigger_mode{false};        ///< true = hardware trigger on Line3
    std::string trigger_source{"Line3"};    ///< Camera input line, e.g. Line2
    int         trigger_delay_us{29};       ///< microseconds
    int         binning{1};                 ///< 1 = full resolution
    int         acquire_timeout_ms{2000};
    double      sync_tolerance_ms{0.0};     ///< max host-arrival difference for a pair (0 = half frame period)
    int         max_consecutive_errors{10}; ///< consecutive grab errors before shutdown
};

/// A raw frame straight from the sensor — BayerRG8, no conversion.
/// Downstream workers choose their own conversion (Gray for SLAM, RGB for Retinify/Preview).
struct RawFrame {
    cv::Mat    image;           ///< CV_8UC1, BayerRG8 pattern (unconverted sensor data)
    uint64_t   frame_id{0};
    uint64_t   timestamp_ns{0}; ///< Spinnaker chunk timestamp (camera clock) in nanoseconds
    std::chrono::steady_clock::time_point received_at;
    uint64_t   host_ns{0};      ///< Host system_clock time at frame arrival (ns since epoch)
};

/// Callback type: called on each synchronized stereo pair
using StereoCallback = std::function<void(const RawFrame & left, const RawFrame & right)>;

/// Grabber statistics (monotonic counters, read from any thread)
struct GrabberStats {
    uint64_t left_frames{0};
    uint64_t right_frames{0};
    uint64_t pairs{0};
    uint64_t dropped_unmatched{0};
    uint64_t incomplete{0};
};

/// Acquires synchronized stereo frames from two BFS Spinnaker cameras.
/// Delivers raw BayerRG8 frames — all pixel format conversion is deferred to consumers.
///
/// Pairing is done on host arrival time (nearest neighbour within sync_tolerance_ms).
/// This works for both modes:
///   - Hardware trigger: both cameras expose on the same pulse, arrive within a few ms.
///   - Continuous: free-running cameras have independent FrameID counters and clocks,
///     so arrival pairing is only an approximation, not exposure synchronization.
class SpinnakerGrabber
{
public:
    explicit SpinnakerGrabber(const CameraConfig & cfg);
    ~SpinnakerGrabber();

    /// Set callback invoked for each synchronized stereo pair.
    void setCallback(StereoCallback cb) { callback_ = std::move(cb); }

    /// Open cameras, configure, begin acquisition on BOTH cameras, then start threads.
    /// When this returns, both cameras are armed (safe to start an external trigger).
    void start();

    /// Stop acquisition and release cameras.
    void stop();

    bool isRunning() const { return running_.load(); }

    GrabberStats stats() const;

    /// Serials actually used (may differ from config if auto-detected).
    const std::string & serialLeft()  const { return cfg_.serial_left; }
    const std::string & serialRight() const { return cfg_.serial_right; }

    /// Effective tolerance in ms used for pairing.
    double syncToleranceMs() const { return sync_tol_ms_; }

private:
    void configureCamera(Spinnaker::CameraPtr cam, bool is_left);
    void grabThread(Spinnaker::CameraPtr cam, bool is_left);
    void syncThread();

    CameraConfig   cfg_;
    StereoCallback callback_;

    Spinnaker::SystemPtr    system_;
    Spinnaker::CameraPtr    cam_left_;
    Spinnaker::CameraPtr    cam_right_;

    // Per-camera raw frame queues for synchronization (single mutex: sync needs both)
    std::deque<RawFrame>    left_queue_;
    std::deque<RawFrame>    right_queue_;
    std::mutex              mtx_;
    std::condition_variable cv_;

    std::thread left_grab_thread_;
    std::thread right_grab_thread_;
    std::thread sync_thread_;

    std::atomic<bool> running_{false};
    bool              started_{false};   ///< threads launched (join needed even if running_ dropped)
    double            sync_tol_ms_{16.0};

    std::atomic<uint64_t> n_left_{0}, n_right_{0}, n_pairs_{0}, n_dropped_{0}, n_incomplete_{0};

    static constexpr std::size_t kQueueDepth = 4;
};

}  // namespace passive_stereo_capture
