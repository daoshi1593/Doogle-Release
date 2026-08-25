#include "instance/runtime/command_slot.hpp"

namespace doogle::runtime {

void CommandSlot::store(const MotionIntent& value) {
    protocol::RobotControlCommand command;
    command.mode = static_cast<std::int8_t>(value.mode);
    command.gait_id = static_cast<std::int8_t>(value.gait_id);
    command.contact = static_cast<std::int8_t>(value.contact);
    command.vel_des = {value.vx, value.vy, value.wz};
    command.pos_des[2] = value.body_height;
    store(command);
}

void CommandSlot::store(const protocol::RobotControlCommand& value) {
    std::lock_guard lock(mutex_);
    latest_ = value;
}

protocol::RobotControlCommand CommandSlot::load() const {
    std::lock_guard lock(mutex_);
    return latest_;
}

}  // namespace doogle::runtime
