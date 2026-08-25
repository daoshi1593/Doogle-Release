#include "instance/runtime/heartbeat.hpp"
namespace doogle::runtime {
protocol::RobotControlCommand Heartbeat::tick() {
    auto command = slot_.load();
    life_count_ = static_cast<std::uint8_t>((life_count_ + 1U) % 128U);
    command.life_count = static_cast<std::int8_t>(life_count_);
    return command;
}
}  // namespace doogle::runtime
