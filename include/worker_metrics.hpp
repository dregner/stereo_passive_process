#pragma once
#include <atomic>
#include <chrono>

namespace passive_stereo_capture {
// Elapsed durations use the monotonic host clock, never camera or ROS clock time.
struct WorkerMetrics {
    std::atomic<double> process_ms{0.0};
    std::atomic<double> receipt_age_ms{0.0};
    void record(std::chrono::steady_clock::time_point start,
                std::chrono::steady_clock::time_point received) {
        const auto finish = std::chrono::steady_clock::now();
        process_ms.store(std::chrono::duration<double, std::milli>(finish - start).count());
        receipt_age_ms.store(std::chrono::duration<double, std::milli>(finish - received).count());
    }
};
}
