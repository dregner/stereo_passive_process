#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "bounded_queue.hpp"

namespace passive_stereo_capture
{

// Keep DDS sends off the processing thread. Retain one pending sample per topic.
template<typename Message>
class QueuedPublisher
{
public:
    explicit QueuedPublisher(typename rclcpp::Publisher<Message>::SharedPtr publisher)
    : publisher_(std::move(publisher)) {}

    ~QueuedPublisher() { stop(); }

    bool enabled() const { return static_cast<bool>(publisher_); }

    bool hasSubscribers() const
    {
        return publisher_ && publisher_->get_subscription_count() > 0;
    }

    void start()
    {
        if (!publisher_ || thread_.joinable()) return;
        running_.store(true);
        thread_ = std::thread(&QueuedPublisher::run, this);
    }

    void stop()
    {
        running_.store(false);
        queue_.shutdown();
        if (thread_.joinable()) thread_.join();
    }

    void push(std::unique_ptr<Message> message) { queue_.push(std::move(message)); }
    uint64_t dropped() const { return queue_.dropped(); }
    uint64_t published() const { return published_.load(); }

private:
    void run()
    {
        rclcpp::Clock clock(RCL_STEADY_TIME);
        const auto logger = rclcpp::get_logger("passive_stereo_capture");
        while (running_.load()) {
            std::unique_ptr<Message> message;
            if (!queue_.pop(message) || !running_.load() || !hasSubscribers()) continue;
            try {
                publisher_->publish(std::move(message));
                ++published_;
            } catch (const std::exception & error) {
                RCLCPP_WARN_THROTTLE(logger, clock, 5000, "Publish failed on %s: %s",
                    publisher_->get_topic_name(), error.what());
            }
        }
    }

    typename rclcpp::Publisher<Message>::SharedPtr publisher_;
    BoundedQueue<std::unique_ptr<Message>> queue_{1};
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> published_{0};
    std::thread thread_;
};

}  // namespace passive_stereo_capture
