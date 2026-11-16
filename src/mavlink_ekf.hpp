#pragma once

#include <deque>
#include <mutex>
#include <ardupilotmega/mavlink.h>
#include "mavlink_worker.hpp"

namespace passive_stereo_capture
{
// poll() runs on the MAVLink worker. poseAt() is called by the SLAM worker.
class MavlinkEkf
{
public:
    explicit MavlinkEkf(const MavlinkWorker::Config & cfg);

    void poll(MavlinkTransport & transport, mavlink_status_t & tx,
              const MavlinkWorker::Clock & clock);
    std::optional<Eigen::Isometry3d> poseAt(uint64_t usec, uint64_t now_usec) const;

private:
    void handle(const mavlink_message_t & packet, int64_t received_usec);

    MavlinkWorker::Config cfg_;
    mavlink_message_t parser_{};
    mavlink_status_t parser_status_{};
    uint64_t last_request_usec_{0};
    int64_t sync_request_usec_{0};
    int64_t last_sync_usec_{0};
    int64_t boot_to_host_usec_{0};
    mutable std::mutex mutex_;
    std::deque<mavlink_local_position_ned_t> positions_;
    std::deque<mavlink_attitude_quaternion_t> attitudes_;
    uint64_t position_received_usec_{0};
    uint64_t attitude_received_usec_{0};
    uint64_t status_received_usec_{0};
    uint16_t ekf_flags_{0};
};
}
