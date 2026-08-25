#include "service/adapter/udp_motion_sink.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "service/protocol/lcm/command_codec.hpp"

namespace doogle::adapters {

UdpMotionSink::UdpMotionSink(std::string host, std::uint16_t port)
    : socket_(::socket(AF_INET, SOCK_DGRAM, 0)), host_(std::move(host)), port_(port) {}

UdpMotionSink::~UdpMotionSink() {
    if (socket_ >= 0) ::close(socket_);
}

void UdpMotionSink::publish(const protocol::RobotControlCommand& command) {
    if (socket_ < 0) return;
    sockaddr_in target{};
    target.sin_family = AF_INET;
    target.sin_port = htons(port_);
    if (::inet_pton(AF_INET, host_.c_str(), &target.sin_addr) != 1) return;
    const auto packet = protocol::encode_command(command);
    (void)::sendto(socket_, packet.data(), packet.size(), 0, reinterpret_cast<sockaddr*>(&target),
                   sizeof(target));
}

}  // namespace doogle::adapters
