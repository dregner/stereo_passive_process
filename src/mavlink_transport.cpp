#include "mavlink_transport.hpp"
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <charconv>
#include <memory>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <poll.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace passive_stereo_capture
{
namespace {
[[noreturn]] void fail(const char * op) { throw std::system_error(errno, std::generic_category(), op); }
int number(const std::string & s, int low, int high) {
    int n = 0;
    auto [end, ec] = std::from_chars(s.data(), s.data() + s.size(), n);
    if (ec != std::errc{} || end != s.data() + s.size() || n < low || n > high)
        throw std::invalid_argument("Invalid MAVLink port/baud: " + s);
    return n;
}
struct Endpoint { std::string host, port; };
Endpoint endpoint(const std::string & s, bool bind) {
    const auto colon = s.rfind(':');
    if (colon == std::string::npos || s.find(':') != colon)
        throw std::invalid_argument("Expected IPv4/hostname:port MAVLink endpoint");
    number(s.substr(colon + 1), bind ? 0 : 1, 65535);
    auto host = s.substr(0, colon);
    if (host.empty() && !bind) throw std::invalid_argument("MAVLink destination host is required");
    return {host, s.substr(colon + 1)};
}
sockaddr_in address(const Endpoint & ep, int type, bool bind) {
    addrinfo hints{}; hints.ai_family = AF_INET; hints.ai_socktype = type;
    hints.ai_flags = bind ? AI_PASSIVE : 0;
    addrinfo * raw = nullptr;
    const int rc = getaddrinfo(ep.host.empty() ? nullptr : ep.host.c_str(), ep.port.c_str(), &hints, &raw);
    if (rc) throw std::runtime_error(std::string("MAVLink address: ") + gai_strerror(rc));
    std::unique_ptr<addrinfo, decltype(&freeaddrinfo)> result(raw, freeaddrinfo);
    return *reinterpret_cast<sockaddr_in *>(result->ai_addr);
}
speed_t baud(int n) {
    switch(n) {
    case 9600: return B9600; case 19200: return B19200; case 38400: return B38400;
    case 57600: return B57600; case 115200: return B115200; case 230400: return B230400;
    case 460800: return B460800; case 921600: return B921600;
    default: throw std::invalid_argument("Unsupported MAVLink serial baud");
    }
}
}
MavlinkTransport::MavlinkTransport(const std::string & url)
{
    try {
        if (url.starts_with("serial://")) {
            const auto body = url.substr(9);
            const auto colon = body.rfind(':');
            if (colon == std::string::npos || body.empty() || body.front() != '/')
                throw std::invalid_argument("Expected serial:///absolute/device:baud");
            const auto speed = baud(number(body.substr(colon + 1), 1, 1000000));
            fd_ = ::open(body.substr(0, colon).c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
            if (fd_ < 0) fail("MAVLink serial open");
            serial_ = true;
            termios opts{};
            if (tcgetattr(fd_, &opts)) fail("MAVLink serial tcgetattr");
            cfmakeraw(&opts);
            opts.c_cflag |= CLOCAL | CREAD;
            opts.c_cflag &= ~(CSTOPB | CRTSCTS);
            if (cfsetispeed(&opts, speed) || cfsetospeed(&opts, speed) || tcsetattr(fd_, TCSANOW, &opts))
                fail("MAVLink serial configure");
            return;
        }
        datagram_ = url.starts_with("udp://");
        if (!datagram_ && !url.starts_with("tcp://"))
            throw std::invalid_argument("MAVLink URL must use udp://, tcp:// or serial://");
        const auto body = url.substr(6);
        const auto at = body.find('@');
        if (datagram_ && (at == std::string::npos || body.find('@', at + 1) != std::string::npos))
            throw std::invalid_argument("Expected udp://bind_host:port@remote_host:port");
        const int type = datagram_ ? SOCK_DGRAM : SOCK_STREAM;
        const auto remote = address(endpoint(datagram_ ? body.substr(at + 1) : body, false), type, false);
        fd_ = socket(AF_INET, type | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd_ < 0) fail("MAVLink socket");
        if (datagram_) {
            const auto local = address(endpoint(body.substr(0, at), true), type, true);
            if (::bind(fd_, reinterpret_cast<const sockaddr *>(&local), sizeof(local))) fail("MAVLink UDP bind");
        } else {
            int yes = 1;
            if (setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes))) fail("MAVLink TCP_NODELAY");
        }
        if (::connect(fd_, reinterpret_cast<const sockaddr *>(&remote), sizeof(remote))) {
            if (datagram_ || errno != EINPROGRESS) fail("MAVLink connect");
            pollfd p{fd_, POLLOUT, 0};
            if (::poll(&p, 1, 1000) <= 0) throw std::runtime_error("MAVLink TCP connect timed out/interrupted");
            int error = 0; socklen_t len = sizeof(error);
            if (getsockopt(fd_, SOL_SOCKET, SO_ERROR, &error, &len)) fail("MAVLink TCP connect status");
            if (error) throw std::system_error(error, std::generic_category(), "MAVLink TCP connect");
        }
    } catch (...) { close(); throw; }
}
MavlinkTransport::~MavlinkTransport() { close(); }
void MavlinkTransport::close() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
void MavlinkTransport::send(const uint8_t * data, std::size_t size)
{
    if (fd_ < 0) throw std::runtime_error("MAVLink transport closed; restart required");
    std::size_t sent = 0;
    while (sent < size) {
        const auto n = serial_ ? ::write(fd_, data + sent, size - sent) :
            ::send(fd_, data + sent, size - sent, MSG_DONTWAIT | MSG_NOSIGNAL);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) {
            const int error = n < 0 ? errno : EIO;
            // A partial stream packet cannot be resumed on a later sample.
            if (sent || (error != EAGAIN && error != EWOULDBLOCK && error != ENOBUFS)) close();
            throw std::system_error(error, std::generic_category(), "MAVLink nonblocking send");
        }
        sent += static_cast<std::size_t>(n);
        if (datagram_ && sent != size) { close(); throw std::runtime_error("Partial MAVLink UDP datagram"); }
    }
}
}
