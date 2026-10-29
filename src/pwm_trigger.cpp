#include "pwm_trigger.hpp"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <utility>

namespace passive_stereo_capture {

PwmTrigger::PwmTrigger(std::string chip, int channel, double frequency_hz,
                       double duty_percent)
    : chip_(std::move(chip)), channel_(channel)
{
    if (chip_.empty() || channel < 0 || !std::isfinite(frequency_hz) ||
        frequency_hz < 0.01 || frequency_hz > 10000.0 ||
        !std::isfinite(duty_percent) || duty_percent <= 0.0 || duty_percent >= 100.0)
        throw std::invalid_argument(
            "PWM requires a chip, channel >= 0, frequency in [0.01, 10000] Hz "
            "and duty cycle in (0, 100) percent");
    period_ns_ = static_cast<uint64_t>(std::llround(1e9 / frequency_hz));
    duty_ns_ = static_cast<uint64_t>(std::llround(period_ns_ * duty_percent / 100.0));
    if (duty_ns_ == 0 || duty_ns_ >= period_ns_)
        throw std::invalid_argument("PWM duty cycle rounds to a constant output");
    channel_path_ = chip_ + "/pwm" + std::to_string(channel_);
}

PwmTrigger::~PwmTrigger() { stop(); }

void PwmTrigger::write(const std::string & path, const std::string & value)
{
    const int fd = ::open(path.c_str(), O_WRONLY | O_TRUNC | O_CLOEXEC);
    if (fd < 0)
        throw std::runtime_error("PWM: cannot open " + path + ": " + std::strerror(errno));
    ssize_t count;
    do { count = ::write(fd, value.data(), value.size()); } while (count < 0 && errno == EINTR);
    const int error = errno;
    const int close_result = ::close(fd);
    if (count != static_cast<ssize_t>(value.size()) || close_result != 0)
        throw std::runtime_error("PWM: cannot write " + value + " to " + path + ": " +
            (count < 0 ? std::string(std::strerror(error)) : "short write or close failure"));
}

void PwmTrigger::start()
{
    if (enabled_) return;
    if (owned_ || std::filesystem::exists(channel_path_))
        throw std::runtime_error("PWM channel already exported: " + channel_path_ +
                                 "; stop its owner and unexport it first");
    write(chip_ + "/export", std::to_string(channel_));
    owned_ = true;
    try {
        // Allow sysfs creation and udev permission rules to finish after export.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        bool ready = false;
        std::string unavailable;
        do {
            ready = true;
            unavailable.clear();
            for (const auto * name : {"period", "duty_cycle", "enable"}) {
                if (::access((channel_path_ + "/" + name).c_str(), W_OK) != 0) {
                    const int error = errno;
                    ready = false;
                    unavailable += std::string(" ") + name + " (" + std::strerror(error) + ")";
                }
            }
            // Some permission rules expose polarity read-only. No write is
            // needed if the fresh channel already has the required polarity.
            std::string polarity;
            std::ifstream(channel_path_ + "/polarity") >> polarity;
            if (polarity != "normal" &&
                ::access((channel_path_ + "/polarity").c_str(), W_OK) != 0) {
                const int error = errno;
                ready = false;
                unavailable += std::string(" polarity (requires normal or write access: ") +
                               std::strerror(error) + ")";
            }
            if (ready) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } while (std::chrono::steady_clock::now() < deadline);
        if (!ready)
            throw std::runtime_error("PWM controls did not become writable: " + channel_path_ +
                                     "; unavailable:" + unavailable +
                                     "; configure sysfs/udev permissions for the capture user");
        // Unexport can retain the old period and duty. Clear that duty before
        // shortening the period, otherwise the kernel rejects duty > period.
        // A fresh Tegra channel has period zero and rejects even zero-duty
        // writes, so it needs the opposite order.
        uint64_t previous_period = 0;
        if (!(std::ifstream(channel_path_ + "/period") >> previous_period))
            throw std::runtime_error("PWM: cannot read current period: " + channel_path_);
        if (previous_period > 0) {
            period_configured_ = true;
            write(channel_path_ + "/duty_cycle", "0");
            write(channel_path_ + "/enable", "0");
        }
        write(channel_path_ + "/period", std::to_string(period_ns_));
        period_configured_ = true;
        write(channel_path_ + "/duty_cycle", "0");
        std::string polarity;
        std::ifstream(channel_path_ + "/polarity") >> polarity;
        if (polarity != "normal") write(channel_path_ + "/polarity", "normal");
        write(channel_path_ + "/duty_cycle", std::to_string(duty_ns_));
        write(channel_path_ + "/enable", "1");
        enabled_ = true;
    } catch (...) {
        stop();
        throw;
    }
}

void PwmTrigger::stop() noexcept
{
    if (!owned_) return;
    // Before a valid period is set, no output was enabled by this owner.
    // Tegra can reject even zero duty/disable writes while the period is zero.
    if (period_configured_) {
        for (const auto * name : {"duty_cycle", "enable"}) {
            try { write(channel_path_ + "/" + name, "0"); }
            catch (const std::exception & e) { std::cerr << e.what() << '\n'; }
        }
    }
    enabled_ = false;
    try {
        write(chip_ + "/unexport", std::to_string(channel_));
        owned_ = false;
        period_configured_ = false;
    } catch (const std::exception & e) { std::cerr << e.what() << '\n'; }
}

}  // namespace passive_stereo_capture
