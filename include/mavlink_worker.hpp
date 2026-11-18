#pragma once

#include <atomic>
#include <thread>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include "mavlink_transport.hpp"
#include "bounded_queue.hpp"
#include "mavlink_vision.hpp"

namespace passive_stereo_capture
{
class MavlinkEkf;

class MavlinkWorker
{
public:
    struct Config {
        std::string bridge_url{"udp://:14555@127.0.0.1:14550"};
        std::string message{"estimate"};
        int system_id{1};
        int component_id{197};
        double max_gap_sec{0.5};
        double map_yaw_deg{0.0};
        double position_variance{0.05};
        double angle_variance{0.1};
        double confidence{100.0};
        bool receive_ekf{false};
        int autopilot_system_id{1};
        double ekf_max_skew_sec{0.15};
        bool send_vision{true};
    };
    using Clock = std::function<uint64_t()>;  // Microseconds, same clock as VisionSample.
    using ErrorSink = std::function<void(const std::string &)>;
    explicit MavlinkWorker(const Config & cfg, Clock clock = {}, ErrorSink errors = {});
    ~MavlinkWorker();
    void push(VisionSample sample);
    bool toggleVision();
    std::optional<Eigen::Isometry3d> ekfPoseAt(uint64_t usec) const;
    void stop();
private:
    struct QueuedVision {
        VisionSample sample;
        uint64_t generation{0};
    };

    void run();
    Clock clock_;
    ErrorSink errors_;
    Config cfg_;
    std::unique_ptr<MavlinkTransport> bridge_;
    std::unique_ptr<MavlinkEkf> ekf_;
    std::mutex vision_mutex_;
    bool send_vision_;
    uint64_t vision_generation_{0};
    BoundedQueue<QueuedVision> queue_{1, true};
    std::atomic<bool> running_{true};
    std::thread thread_;
};
}  // namespace passive_stereo_capture
