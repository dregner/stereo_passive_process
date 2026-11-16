#include "mavlink_ekf.hpp"

#include <array>
#include <cmath>

namespace passive_stereo_capture
{
namespace
{
constexpr uint64_t kStreamTimeoutUsec = 2'000'000;
constexpr int64_t kSyncTimeoutUsec = 3'000'000;

// time_boot_ms is a wrapping 32-bit counter. The signed difference is valid
// for samples within half a counter period of the requested capture time.
template<typename Sample>
const Sample * nearest(const std::deque<Sample> & samples, uint32_t boot_ms, double max_skew_ms)
{
    const Sample * result = nullptr;
    for (auto it = samples.rbegin(); it != samples.rend(); ++it) {
        const auto skew = std::abs(static_cast<double>(static_cast<int32_t>(boot_ms - it->time_boot_ms)));
        if (skew < max_skew_ms || (!result && skew == max_skew_ms)) {
            max_skew_ms = skew;
            result = &*it;
        }
    }
    return result;
}
}

MavlinkEkf::MavlinkEkf(const MavlinkWorker::Config & cfg) : cfg_(cfg) {}

void MavlinkEkf::poll(MavlinkTransport & transport, mavlink_status_t & tx,
                      const MavlinkWorker::Clock & clock)
{
    std::array<uint8_t, 65536> bytes;
    for (int read = 0; read < 16; ++read) {
        const auto size = transport.receive(bytes.data(), bytes.size());
        if (!size) break;
        const auto received = clock();
        for (size_t i = 0; i < size; ++i) {
            mavlink_message_t packet{};
            mavlink_status_t status{};
            if (mavlink_frame_char_buffer(&parser_, &parser_status_, bytes[i], &packet, &status) ==
                MAVLINK_FRAMING_OK && packet.sysid == static_cast<unsigned>(cfg_.autopilot_system_id) &&
                packet.compid == MAV_COMP_ID_AUTOPILOT1) {
                handle(packet, static_cast<int64_t>(received));
            }
        }
    }

    const auto now = clock();
    if (!now) return;
    auto send = [&](const mavlink_message_t & packet) {
        const auto size = mavlink_msg_to_send_buffer(bytes.data(), &packet);
        transport.send(bytes.data(), size);
    };
    if (now - static_cast<uint64_t>(sync_request_usec_) >= 1'000'000 || !sync_request_usec_) {
        sync_request_usec_ = static_cast<int64_t>(now);
        mavlink_timesync_t request{};
        request.ts1 = sync_request_usec_ * 1000;
        mavlink_message_t packet{};
        mavlink_msg_timesync_encode_status(cfg_.system_id, cfg_.component_id, &tx, &packet, &request);
        send(packet);
    }
    if (last_request_usec_ && now - last_request_usec_ < kStreamTimeoutUsec) return;
    last_request_usec_ = now;
    auto requestStream = [&](uint32_t id, float interval_usec, uint64_t last_received) {
        if (last_received && now - last_received < kStreamTimeoutUsec) return;
        mavlink_command_long_t request{};
        request.target_system = cfg_.autopilot_system_id;
        request.target_component = MAV_COMP_ID_AUTOPILOT1;
        request.command = MAV_CMD_SET_MESSAGE_INTERVAL;
        request.param1 = static_cast<float>(id);
        request.param2 = interval_usec;
        mavlink_message_t packet{};
        mavlink_msg_command_long_encode_status(cfg_.system_id, cfg_.component_id, &tx, &packet, &request);
        send(packet);
    };
    requestStream(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50'000, position_received_usec_);
    requestStream(MAVLINK_MSG_ID_ATTITUDE_QUATERNION, 50'000, attitude_received_usec_);
    requestStream(MAVLINK_MSG_ID_EKF_STATUS_REPORT, 200'000, status_received_usec_);
}

void MavlinkEkf::handle(const mavlink_message_t & packet, int64_t received_usec)
{
    std::lock_guard lock(mutex_);
    switch (packet.msgid) {
    case MAVLINK_MSG_ID_TIMESYNC: {
        mavlink_timesync_t sync;
        mavlink_msg_timesync_decode(&packet, &sync);
        const auto rtt_usec = received_usec - sync_request_usec_;
        if (sync.tc1 <= 0 || sync.ts1 != sync_request_usec_ * 1000 ||
            !sync_request_usec_ || rtt_usec < 0 || rtt_usec > 100'000) break;
        const auto offset = sync_request_usec_ + rtt_usec / 2 - sync.tc1 / 1000;
        if (last_sync_usec_ && std::abs(offset - boot_to_host_usec_) > 200'000) {
            positions_.clear();
            attitudes_.clear();
            status_received_usec_ = 0;
        }
        boot_to_host_usec_ = offset;
        last_sync_usec_ = received_usec;
        break;
    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
        mavlink_local_position_ned_t position;
        mavlink_msg_local_position_ned_decode(&packet, &position);
        const Eigen::Vector3d p(position.x, position.y, position.z);
        const Eigen::Vector3d v(position.vx, position.vy, position.vz);
        if (!p.allFinite() || !v.allFinite()) break;
        position_received_usec_ = received_usec;
        positions_.push_back(position);
        if (positions_.size() > 64) positions_.pop_front();
        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION: {
        mavlink_attitude_quaternion_t attitude;
        mavlink_msg_attitude_quaternion_decode(&packet, &attitude);
        const Eigen::Quaterniond q(attitude.q1, attitude.q2, attitude.q3, attitude.q4);
        if (!q.coeffs().allFinite() || std::abs(q.squaredNorm() - 1.0) > 0.02) break;
        attitude_received_usec_ = received_usec;
        attitudes_.push_back(attitude);
        if (attitudes_.size() > 64) attitudes_.pop_front();
        break;
    }
    case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
        mavlink_ekf_status_report_t status;
        mavlink_msg_ekf_status_report_decode(&packet, &status);
        status_received_usec_ = received_usec;
        ekf_flags_ = status.flags;
        break;
    }
    }
}

std::optional<Eigen::Isometry3d> MavlinkEkf::poseAt(uint64_t usec, uint64_t now_usec) const
{
    std::lock_guard lock(mutex_);
    if (!usec || usec > now_usec || now_usec - usec > cfg_.max_gap_sec * 1e6 ||
        !last_sync_usec_ || now_usec - static_cast<uint64_t>(last_sync_usec_) > kSyncTimeoutUsec ||
        !status_received_usec_ || now_usec - status_received_usec_ > kStreamTimeoutUsec ||
        !(ekf_flags_ & EKF_ATTITUDE) || !(ekf_flags_ & (EKF_POS_HORIZ_REL | EKF_POS_HORIZ_ABS)) ||
        (ekf_flags_ & (EKF_CONST_POS_MODE | EKF_UNINITIALIZED))) return std::nullopt;

    const auto boot_usec = static_cast<int64_t>(usec) - boot_to_host_usec_;
    if (boot_usec < 0) return std::nullopt;
    const auto boot_ms = static_cast<uint32_t>(boot_usec / 1000);
    const auto max_skew_ms = cfg_.ekf_max_skew_sec * 1000;
    const auto * p = nearest(positions_, boot_ms, max_skew_ms);
    const auto * q = nearest(attitudes_, boot_ms, max_skew_ms);
    if (!p || !q) return std::nullopt;

    const double position_dt = static_cast<int32_t>(boot_ms - p->time_boot_ms) * 1e-3;
    const double attitude_dt = static_cast<int32_t>(boot_ms - q->time_boot_ms) * 1e-3;
    Eigen::Quaterniond attitude(q->q1, q->q2, q->q3, q->q4);
    attitude.normalize();
    const Eigen::Vector3d angular_velocity(q->rollspeed, q->pitchspeed, q->yawspeed);
    const double angular_speed = angular_velocity.norm();
    if (std::isfinite(angular_speed) && angular_speed > 0) {
        attitude = attitude * Eigen::AngleAxisd(
            attitude_dt * angular_speed, angular_velocity / angular_speed);
    }
    Eigen::Matrix3d ned_to_enu;
    ned_to_enu << 0, 1, 0,
                  1, 0, 0,
                  0, 0, -1;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = ned_to_enu *
        (Eigen::Vector3d(p->x, p->y, p->z) + position_dt * Eigen::Vector3d(p->vx, p->vy, p->vz));
    pose.linear() = ned_to_enu * attitude.toRotationMatrix() * fluToFrd();
    return pose;
}
}
