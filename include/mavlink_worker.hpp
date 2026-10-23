#pragma once

#include <atomic>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <mavconn/interface.hpp>
#include "bounded_queue.hpp"
#include "mavlink_vision.hpp"

namespace passive_stereo_capture
{
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
    };
    MavlinkWorker(rclcpp::Node * node, const Config & cfg);
    ~MavlinkWorker();
    void push(VisionSample sample);
    void stop();
private:
    void run();
    rclcpp::Node * node_;
    Config cfg_;
    mavconn::MAVConnInterface::Ptr bridge_;
    BoundedQueue<VisionSample> queue_{1, true};
    std::atomic<bool> running_{true};
    std::thread thread_;
};
}  // namespace passive_stereo_capture
