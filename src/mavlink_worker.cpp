#include "mavlink_worker.hpp"
#include "mavlink_ekf.hpp"

#include <cmath>
#include <stdexcept>
#include <numbers>
#include <ardupilotmega/mavlink.h>

namespace passive_stereo_capture
{
MavlinkWorker::MavlinkWorker(const Config & cfg, Clock clock, ErrorSink errors)
: clock_(std::move(clock)), errors_(std::move(errors)), cfg_(cfg), send_vision_(cfg.send_vision)
{
    if (cfg.message != "estimate" && cfg.message != "delta")
        throw std::invalid_argument("mavlink_message must be estimate or delta");
    if (cfg.system_id < 1 || cfg.system_id > 255 || cfg.component_id < 1 || cfg.component_id > 255)
        throw std::invalid_argument("MAVLink source IDs must be in [1, 255]");
    if (cfg.bridge_url.empty() || !std::isfinite(cfg.max_gap_sec) || cfg.max_gap_sec <= 0 ||
        !std::isfinite(cfg.map_yaw_deg) || !std::isfinite(cfg.position_variance) ||
        cfg.position_variance <= 0 || !std::isfinite(cfg.angle_variance) || cfg.angle_variance <= 0 ||
        !std::isfinite(cfg.confidence) || cfg.confidence <= 0 || cfg.confidence > 100)
        throw std::invalid_argument("Invalid MAVLink URL, timing, alignment, uncertainty or confidence");
    if (!clock_) clock_ = [] {
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    };
    if (cfg.receive_ekf) {
        if (cfg.autopilot_system_id < 1 || cfg.autopilot_system_id > 255 ||
            !std::isfinite(cfg.ekf_max_skew_sec) || cfg.ekf_max_skew_sec <= 0)
            throw std::invalid_argument("Invalid MAVLink autopilot system ID or EKF timestamp tolerance");
        ekf_ = std::make_unique<MavlinkEkf>(cfg);
    }
    bridge_ = std::make_unique<MavlinkTransport>(cfg.bridge_url, cfg.receive_ekf);
    thread_ = std::thread(&MavlinkWorker::run, this);
}

MavlinkWorker::~MavlinkWorker() { stop(); }

void MavlinkWorker::push(VisionSample sample)
{
    std::lock_guard lock(vision_mutex_);
    if (send_vision_) queue_.push({std::move(sample), vision_generation_});
}

bool MavlinkWorker::toggleVision()
{
    std::lock_guard lock(vision_mutex_);
    ++vision_generation_;
    send_vision_ = !send_vision_;
    return send_vision_;
}

std::optional<Eigen::Isometry3d> MavlinkWorker::ekfPoseAt(uint64_t usec) const
{
    return ekf_ ? ekf_->poseAt(usec, clock_()) : std::nullopt;
}

void MavlinkWorker::stop()
{
    running_.store(false);
    queue_.shutdown();
    if (thread_.joinable()) thread_.join();
    if (bridge_) bridge_->close();
}

void MavlinkWorker::run()
{
    mavlink_status_t tx_status{};  // Private sequence state, MAVLink 2 by default.
    VisionSample previous;
    bool have_previous = false;
    uint8_t reset_counter = 0;
    uint64_t previous_generation = 0;
    const double yaw = cfg_.map_yaw_deg * std::numbers::pi / 180.0;
    while (running_.load()) {
        if (ekf_) {
            try {
                ekf_->poll(*bridge_, tx_status, clock_);
            } catch (const std::exception & ex) {
                if (errors_) errors_(ex.what());
            }
        }
        QueuedVision pending;
        const auto timeout = std::chrono::milliseconds(ekf_ ? 5 : 200);
        if (!queue_.pop(pending, timeout) || !running_.load()) continue;
        // Serialize writes with the service and discard samples queued before a toggle.
        std::unique_lock lock(vision_mutex_);
        if (!send_vision_ || pending.generation != vision_generation_) continue;
        if (pending.generation != previous_generation) {
            if (have_previous) ++reset_counter;
            have_previous = false;
            previous_generation = pending.generation;
        }
        const auto & sample = pending.sample;
        if (sample.usec == 0 || !sample.position.allFinite() ||
            !sample.orientation.coeffs().allFinite() ||
            std::abs(sample.orientation.norm() - 1.0) > 1e-3) {
            if (have_previous) ++reset_counter;
            have_previous = false;
            continue;
        }
        const auto now_usec = clock_();
        const double age = static_cast<double>(now_usec) / 1e6 - sample.usec / 1e6;
        if (age < 0 || age > cfg_.max_gap_sec) {
            if (have_previous) ++reset_counter;
            have_previous = false;
            continue;
        }
        const bool discontinuity = have_previous &&
            (sample.epoch != previous.epoch || sample.usec <= previous.usec ||
             (sample.usec - previous.usec) / 1e6 > cfg_.max_gap_sec);
        if (discontinuity) { ++reset_counter; have_previous = false; }
        try {
            auto send = [&](const mavlink_message_t & packet) {
                uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
                const auto size = mavlink_msg_to_send_buffer(bytes, &packet);
                bridge_->send(bytes, size);
            };
            const auto rotation = visionRotation(sample, yaw);
            if (cfg_.message == "estimate") {
                mavlink_vision_position_estimate_t msg{};
                msg.usec = sample.usec;
                const auto p = visionPosition(sample, yaw);
                msg.x = p.x(); msg.y = p.y(); msg.z = 0;
                const auto rpy = visionRpy(rotation);
                msg.roll = rpy[0]; msg.pitch = rpy[1]; msg.yaw = rpy[2];
                msg.covariance[0] = msg.covariance[6] = cfg_.position_variance;
                msg.covariance[11] = cfg_.position_variance;  // Disable Z fusion in EK3 sources.
                msg.covariance[15] = msg.covariance[18] = msg.covariance[20] = cfg_.angle_variance;
                msg.reset_counter = reset_counter;
                mavlink_message_t packet{};
                mavlink_msg_vision_position_estimate_encode_status(
                    cfg_.system_id, cfg_.component_id, &tx_status, &packet, &msg);
                send(packet);
            } else if (have_previous) {
                mavlink_vision_position_delta_t msg{};
                msg.time_usec = sample.usec;
                msg.time_delta_usec = sample.usec - previous.usec;
                const Eigen::Vector3d dp = rotation.transpose() *
                    (visionPosition(sample, yaw) - visionPosition(previous, yaw));
                const Eigen::Vector3d dr = visionRotationVector(
                    rotation.transpose() * visionRotation(previous, yaw));
                for (int i = 0; i < 3; ++i) {
                    msg.position_delta[i] = dp[i];
                    msg.angle_delta[i] = dr[i];
                }
                msg.confidence = cfg_.confidence;
                mavlink_message_t packet{};
                mavlink_msg_vision_position_delta_encode_status(
                    cfg_.system_id, cfg_.component_id, &tx_status, &packet, &msg);
                send(packet);
            }
            previous = sample;
            have_previous = true;
        } catch (const std::exception & ex) {
            // Never bridge across a failed send with an unreported delta.
            have_previous = false;
            ++reset_counter;
            lock.unlock();
            if (errors_) errors_(ex.what());
        }
    }
}
}  // namespace passive_stereo_capture
