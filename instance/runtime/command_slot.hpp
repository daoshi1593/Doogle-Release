#pragma once

#include <mutex>

#include "service/domain/motion_intent.hpp"
#include "service/protocol/robot_control_cmd.hpp"

namespace doogle::runtime {

class CommandSlot {
public:
    void store(const MotionIntent& value);
    void store(const protocol::RobotControlCommand& value);
    [[nodiscard]] protocol::RobotControlCommand load() const;

private:
    mutable std::mutex mutex_;
    protocol::RobotControlCommand latest_{};
};

}  // namespace doogle::runtime
