#pragma once

#include <cstdint>
#include <vector>

#include "service/protocol/robot_control_cmd.hpp"

namespace doogle::protocol {

std::vector<std::uint8_t> encode_command(const RobotControlCommand&);
RobotControlCommand decode_command(const std::vector<std::uint8_t>&);
std::uint64_t command_fingerprint();

}  // namespace doogle::protocol
