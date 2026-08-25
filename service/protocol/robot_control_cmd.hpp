#pragma once

#include <array>
#include <cstdint>

namespace doogle::protocol {

struct RobotControlCommand {
    std::int8_t mode{0};
    std::int8_t gait_id{0};
    std::int8_t contact{0};
    std::int8_t life_count{0};
    std::array<float, 3> vel_des{};
    std::array<float, 3> rpy_des{};
    std::array<float, 3> pos_des{};
    std::array<float, 6> acc_des{};
    std::array<float, 3> ctrl_point{};
    std::array<float, 6> foot_pose{};
    std::array<float, 2> step_height{};
    std::int32_t value{0};
    std::int32_t duration{0};
};

}  // namespace doogle::protocol
