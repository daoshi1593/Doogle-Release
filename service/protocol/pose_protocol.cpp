#include "service/protocol/pose_protocol.hpp"

#include <algorithm>
#include <cstring>

namespace doogle::protocol {
namespace {
float read_float_be(const std::vector<std::uint8_t>& bytes, std::size_t& at) {
    if (at + 4 > bytes.size()) return 0.0F;
    std::uint32_t raw = (std::uint32_t(bytes[at]) << 24) | (std::uint32_t(bytes[at + 1]) << 16) |
                        (std::uint32_t(bytes[at + 2]) << 8) | std::uint32_t(bytes[at + 3]);
    at += 4;
    float result{};
    std::memcpy(&result, &raw, sizeof(result));
    return result;
}
std::int64_t read_i64_be(const std::vector<std::uint8_t>& bytes, std::size_t& at) {
    std::uint64_t raw = 0;
    for (int i = 0; i < 8; ++i) raw = (raw << 8) | bytes[at++];
    return static_cast<std::int64_t>(raw);
}
}  // namespace

std::optional<FramedPayload> parse_lc02_frame(const std::vector<std::uint8_t>& packet,
                                              const std::string& expected_channel) {
    if (packet.size() < 10 || packet[0] != 'L' || packet[1] != 'C' || packet[2] != '0' || packet[3] != '2') {
        return std::nullopt;
    }
    const auto nul = std::find(packet.begin() + 8, packet.end(), std::uint8_t{0});
    if (nul == packet.end()) return std::nullopt;
    const std::string channel(packet.begin() + 8, nul);
    if (!expected_channel.empty() && channel != expected_channel) return std::nullopt;
    const std::uint32_t sequence = (std::uint32_t(packet[4]) << 24) | (std::uint32_t(packet[5]) << 16) |
                                    (std::uint32_t(packet[6]) << 8) | std::uint32_t(packet[7]);
    return FramedPayload{sequence, channel, {nul + 1, packet.end()}};
}

std::optional<Pose> decode_pose_payload(const std::vector<std::uint8_t>& payload) {
    if (payload.size() != 76) return std::nullopt;
    Pose pose;
    // global_to_robot is an LCM payload: the first eight bytes are its fingerprint.
    std::size_t at = 8;
    for (auto& value : pose.xyz) value = read_float_be(payload, at);
    for (auto& value : pose.vxyz) value = read_float_be(payload, at);
    for (auto& value : pose.rpy) value = read_float_be(payload, at);
    for (auto& value : pose.omega_body) value = read_float_be(payload, at);
    for (auto& value : pose.v_body) value = read_float_be(payload, at);
    pose.timestamp = read_i64_be(payload, at);
    return pose;
}

}  // namespace doogle::protocol
