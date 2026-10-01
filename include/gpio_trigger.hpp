#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <cstdint>

namespace passive_stereo_capture
{

/// Generates a hardware PWM trigger signal on a Jetson GPIO line using libgpiod.
/// The signal is a 50% duty-cycle square wave at the specified frequency.
/// This triggers both BFS cameras simultaneously via their hardware trigger input (Line3).
class GpioTrigger
{
public:
    /// @param chip_name   GPIO chip device (e.g. "gpiochip0" for Jetson Orin)
    /// @param line_offset GPIO line number on that chip
    /// @param freq_hz     Trigger frequency in Hz (e.g. 30.0)
    GpioTrigger(const std::string & chip_name,
                unsigned int        line_offset,
                double              freq_hz);
    ~GpioTrigger();

    /// Start generating the PWM signal in a background thread.
    void start();

    /// Stop the PWM signal and join the background thread.
    void stop();

    bool isRunning() const { return running_.load(); }

private:
    void run();

    std::string   chip_name_;
    unsigned int  line_offset_;
    double        freq_hz_;
    std::thread   thread_;
    std::atomic<bool> running_{false};
};

}  // namespace passive_stereo_capture
