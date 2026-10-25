#pragma once

#include <cstdint>
#include <string>

namespace passive_stereo_capture {

// Owns a sysfs PWM channel; the peripheral generates pulses without a timing thread.
class PwmTrigger {
public:
    PwmTrigger(std::string chip, int channel, double frequency_hz, double duty_percent);
    ~PwmTrigger();
    PwmTrigger(const PwmTrigger &) = delete;
    PwmTrigger & operator=(const PwmTrigger &) = delete;

    void start();
    void stop() noexcept;

private:
    void write(const std::string & path, const std::string & value);
    std::string chip_;
    std::string channel_path_;
    int channel_;
    uint64_t period_ns_;
    uint64_t duty_ns_;
    bool owned_{false};
    bool enabled_{false};
};

}  // namespace passive_stereo_capture
