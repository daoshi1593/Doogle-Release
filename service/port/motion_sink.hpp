#pragma once

#include "service/protocol/robot_control_cmd.hpp"

namespace doogle::ports {

class MotionSink {
public:
    virtual ~MotionSink() = default;
    virtual void publish(const protocol::RobotControlCommand&) = 0;
};

}  // namespace doogle::ports
