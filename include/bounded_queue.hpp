#pragma once

#include <queue>
#include <cstdint>
#include <stdexcept>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>

namespace passive_stereo_capture
{

/// Thread-safe bounded queue.
///
/// drop_oldest=true  → evict oldest on full (best for preview/disparity: process latest)
/// drop_oldest=false → discard incoming on full (preserves queued older work)
template<typename T>
class BoundedQueue
{
public:
    explicit BoundedQueue(std::size_t max_size, bool drop_oldest = true)
    : max_size_(max_size), drop_oldest_(drop_oldest)
    {
        if (max_size_ == 0) throw std::invalid_argument("queue capacity must be positive");
    }

    void push(T item)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (shutdown_.load()) return;
            if (queue_.size() >= max_size_) {
                dropped_++;
                if (drop_oldest_) {
                    queue_.pop();
                } else {
                    return;
                }
            }
            queue_.push(std::move(item));
        }
        cv_.notify_one();
    }

    bool pop(T & item,
             std::chrono::milliseconds timeout = std::chrono::milliseconds(200))
    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!cv_.wait_for(lock, timeout,
                [this] { return !queue_.empty() || shutdown_.load(); })) {
            return false;
        }
        if (shutdown_.load() && queue_.empty()) return false;
        item = std::move(queue_.front());
        queue_.pop();
        return true;
    }

    uint64_t dropped() const { return dropped_.load(); }

    void shutdown()
    {
        shutdown_.store(true);
        cv_.notify_all();
    }

    std::size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    std::queue<T>           queue_;
    mutable std::mutex      mutex_;
    std::condition_variable cv_;
    std::size_t             max_size_;
    bool                    drop_oldest_;
    std::atomic<bool>       shutdown_{false};
    std::atomic<uint64_t>   dropped_{0};
};

}  // namespace passive_stereo_capture
