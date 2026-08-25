#include "instance/runtime/udp_command_receiver.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <vector>

#include "service/protocol/lcm/command_codec.hpp"

namespace doogle::runtime {

UdpCommandReceiver::UdpCommandReceiver(std::string bind_host, std::uint16_t port)
    : bind_host_(std::move(bind_host)), port_(port) {}

UdpCommandReceiver::~UdpCommandReceiver() { close(); }

bool UdpCommandReceiver::open() {
    close();
    if (port_ == 0) return false;
    socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0) return false;
    const int flags = ::fcntl(socket_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(socket_, F_SETFL, flags | O_NONBLOCK) < 0) {
        close();
        return false;
    }
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port_);
    if (::inet_pton(AF_INET, bind_host_.c_str(), &address.sin_addr) != 1 ||
        ::bind(socket_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
        close();
        return false;
    }
    return true;
}

void UdpCommandReceiver::close() {
    if (socket_ >= 0) ::close(socket_);
    socket_ = -1;
}

std::optional<protocol::RobotControlCommand> UdpCommandReceiver::poll() {
    if (socket_ < 0) return std::nullopt;
    std::optional<protocol::RobotControlCommand> latest;
    std::array<std::uint8_t, 2048> buffer{};
    while (true) {
        const ssize_t size = ::recv(socket_, buffer.data(), buffer.size(), 0);
        if (size < 0) break;
        ++health_.packets;
        try {
            std::vector<std::uint8_t> packet(buffer.begin(), buffer.begin() + size);
            latest = protocol::decode_command(packet);
            ++health_.accepted;
        } catch (...) {
            ++health_.decode_errors;
        }
    }
    return latest;
}

}  // namespace doogle::runtime
