#include "gpio_trigger.hpp"

#include <stdexcept>
#include <chrono>
#include <cmath>
#include <thread>
#include <iostream>

#ifdef HAVE_GPIOD
// libgpiod C API (works on Jetson with libgpiod-dev installed)
extern "C" {
#include <gpiod.h>
}
#endif

namespace passive_stereo_capture
{

GpioTrigger::GpioTrigger(const std::string & chip_name,
                          unsigned int        line_offset,
                          double              freq_hz)
: chip_name_(chip_name),
  line_offset_(line_offset),
  freq_hz_(freq_hz)
{}

GpioTrigger::~GpioTrigger()
{
    stop();
}

void GpioTrigger::start()
{
    if (running_.load()) return;
    running_.store(true);
    thread_ = std::thread(&GpioTrigger::run, this);
}

void GpioTrigger::stop()
{
    running_.store(false);
    if (thread_.joinable()) {
        thread_.join();
    }
}

void GpioTrigger::run()
{
#ifdef HAVE_GPIOD
    // Open GPIO chip (support both "/dev/gpiochipX" and "gpiochipX")
    gpiod_chip * chip = nullptr;
    if (chip_name_.rfind("/dev/", 0) == 0) {
        chip = gpiod_chip_open(chip_name_.c_str());
    } else {
        chip = gpiod_chip_open_by_name(chip_name_.c_str());
        if (!chip) {
            chip = gpiod_chip_open(("/dev/" + chip_name_).c_str());
        }
    }
    if (!chip) {
        throw std::runtime_error(
            "GpioTrigger: cannot open GPIO chip '" + chip_name_ + "'");
    }

    gpiod_line * line = gpiod_chip_get_line(chip, line_offset_);
    if (!line) {
        gpiod_chip_close(chip);
        throw std::runtime_error(
            "GpioTrigger: cannot get GPIO line " + std::to_string(line_offset_));
    }

    if (gpiod_line_request_output(line, "passive_stereo_pwm", 0) < 0) {
        gpiod_chip_close(chip);
        throw std::runtime_error(
            "GpioTrigger: cannot request GPIO line " +
            std::to_string(line_offset_) + " as output");
    }

    // Period and half-period in microseconds
    const long period_us   = static_cast<long>(std::round(1'000'000.0 / freq_hz_));
    const long high_us     = period_us / 2;
    const long low_us      = period_us - high_us;
    using us = std::chrono::microseconds;

    while (running_.load()) {
        gpiod_line_set_value(line, 1);
        std::this_thread::sleep_for(us(high_us));
        gpiod_line_set_value(line, 0);
        std::this_thread::sleep_for(us(low_us));
    }

    // Ensure line is low when stopped
    gpiod_line_set_value(line, 0);
    gpiod_line_release(line);
    gpiod_chip_close(chip);
#else
    std::cerr << "GpioTrigger: libgpiod not compiled in (HAVE_GPIOD not defined). Hardware trigger simulated or inactive.\n";
    const long period_us = static_cast<long>(std::round(1'000'000.0 / freq_hz_));
    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::microseconds(period_us));
    }
#endif
}

}  // namespace passive_stereo_capture

