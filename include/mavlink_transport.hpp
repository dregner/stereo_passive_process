#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

namespace passive_stereo_capture
{
// POSIX transport, no ROS, event loop or application-level transmit queue.
class MavlinkTransport {
public:
    explicit MavlinkTransport(const std::string & url);
    ~MavlinkTransport();
    MavlinkTransport(const MavlinkTransport &) = delete;
    MavlinkTransport & operator=(const MavlinkTransport &) = delete;
    void send(const uint8_t * data, std::size_t size);
    void close();
private:
    int fd_{-1};
    bool serial_{false};
    bool datagram_{false};
};
}
