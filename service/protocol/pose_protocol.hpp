#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace doogle::protocol {

struct Pose {
    std::array<float, 3> xyz{};
    std::array<float, 3> vxyz{};
    std::array<float, 3> rpy{};
    std::array<float, 3> omega_body{};
    std::array<float, 3> v_body{};
    std::int64_t timestamp{0};
};

struct FramedPayload {
    std::uint32_t sequence{0};
    std::string channel;
    std::vector<std::uint8_t> payload;
};

[[nodiscard]] std::optional<FramedPayload> parse_lc02_frame(
    const std::vector<std::uint8_t>&, const std::string& expected_channel = {});
[[nodiscard]] std::optional<Pose> decode_pose_payload(const std::vector<std::uint8_t>&);

}  // namespace doogle::protocol
