#pragma once
#include <cmath>
namespace passive_stereo_capture {
// Keep a scheduled deadline rather than restarting the period at each frame.
// This avoids reducing 15 Hz to 10 Hz when 30 Hz arrival timestamps have jitter.
class RateLimiter {
public:
    bool ready(double now, double hz) {
        if (!std::isfinite(now) || !std::isfinite(hz) || hz <= 0) return false;
        const double period = 1.0 / hz;
        if (!initialized_ || now < previous_) {
            initialized_ = true;
            next_ = now;
        }
        previous_ = now;
        if (now < next_) return false;
        next_ += (std::floor((now - next_) / period) + 1.0) * period;
        return true;
    }
private:
    bool initialized_{false};
    double previous_{0}, next_{0};
};
}
