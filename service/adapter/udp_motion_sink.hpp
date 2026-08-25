#pragma once

#include <cstdint>
#include <string>

#include "service/port/motion_sink.hpp"

namespace doogle::adapters {

class UdpMotionSink final : public ports::MotionSink {
public:
    UdpMotionSink(std::string host, std::uint16_t port);
    ~UdpMotionSink() override;
    void publish(const protocol::RobotControlCommand&) override;

private:
    int socket_{-1};
    std::string host_;
    std::uint16_t port_;
};

}  // namespace doogle::adapters
