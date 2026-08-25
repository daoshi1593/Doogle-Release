#pragma once

#include <cstdint>
#include <chrono>
#include "service/protocol/robot_control_cmd.hpp"
#include "instance/runtime/command_slot.hpp"

namespace doogle::runtime {
inline constexpr std::chrono::milliseconds kHeartbeatPeriod{5};
class Heartbeat {
public:
    explicit Heartbeat(const CommandSlot& slot) : slot_(slot) {}
    [[nodiscard]] protocol::RobotControlCommand tick();
private:
    const CommandSlot& slot_;
    std::uint8_t life_count_{0};
};
}  // namespace doogle::runtime
