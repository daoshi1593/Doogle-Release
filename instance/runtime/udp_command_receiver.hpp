#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "service/protocol/robot_control_cmd.hpp"

namespace doogle::runtime {

struct UdpCommandReceiverHealth {
    std::uint64_t packets{};
    std::uint64_t accepted{};
    std::uint64_t decode_errors{};
};

class UdpCommandReceiver {
public:
    UdpCommandReceiver(std::string bind_host, std::uint16_t port);
    ~UdpCommandReceiver();
    UdpCommandReceiver(const UdpCommandReceiver&) = delete;
    UdpCommandReceiver& operator=(const UdpCommandReceiver&) = delete;

    [[nodiscard]] bool open();
    void close();
    [[nodiscard]] std::optional<protocol::RobotControlCommand> poll();
    [[nodiscard]] UdpCommandReceiverHealth health() const { return health_; }

private:
    std::string bind_host_;
    std::uint16_t port_{};
    int socket_{-1};
    UdpCommandReceiverHealth health_{};
};

}  // namespace doogle::runtime
