#include "service/protocol/lcm/command_codec.hpp"

#include <cstring>
#include <stdexcept>

namespace doogle::protocol {
namespace {
constexpr std::uint64_t kFingerprint = 0x8ed6c3c52d5f2deaULL;
constexpr std::size_t kPayloadSize = 8 + 4 + 26 * 4 + 8;

void put_u32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>(value >> 24));
    out.push_back(static_cast<std::uint8_t>(value >> 16));
    out.push_back(static_cast<std::uint8_t>(value >> 8));
    out.push_back(static_cast<std::uint8_t>(value));
}
void put_float(std::vector<std::uint8_t>& out, float value) {
    std::uint32_t bits{};
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    put_u32(out, bits);
}
std::uint32_t get_u32(const std::vector<std::uint8_t>& in, std::size_t& at) {
    if (at + 4 > in.size()) throw std::invalid_argument("truncated command packet");
    const auto value = (std::uint32_t(in[at]) << 24) | (std::uint32_t(in[at + 1]) << 16) |
                       (std::uint32_t(in[at + 2]) << 8) | std::uint32_t(in[at + 3]);
    at += 4;
    return value;
}
float get_float(const std::vector<std::uint8_t>& in, std::size_t& at) {
    const auto bits = get_u32(in, at);
    float value{};
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}
template <std::size_t N> void put_array(std::vector<std::uint8_t>& out, const std::array<float, N>& a) {
    for (const auto value : a) put_float(out, value);
}
template <std::size_t N> void get_array(const std::vector<std::uint8_t>& in, std::size_t& at, std::array<float, N>& a) {
    for (auto& value : a) value = get_float(in, at);
}
}  // namespace

std::uint64_t command_fingerprint() { return kFingerprint; }

std::vector<std::uint8_t> encode_command(const RobotControlCommand& c) {
    std::vector<std::uint8_t> out;
    out.reserve(kPayloadSize);
    for (int shift = 56; shift >= 0; shift -= 8) out.push_back(static_cast<std::uint8_t>(kFingerprint >> shift));
    out.insert(out.end(), {static_cast<std::uint8_t>(c.mode), static_cast<std::uint8_t>(c.gait_id),
                           static_cast<std::uint8_t>(c.contact), static_cast<std::uint8_t>(c.life_count)});
    put_array(out, c.vel_des); put_array(out, c.rpy_des); put_array(out, c.pos_des);
    put_array(out, c.acc_des); put_array(out, c.ctrl_point); put_array(out, c.foot_pose);
    put_array(out, c.step_height);
    put_u32(out, static_cast<std::uint32_t>(c.value)); put_u32(out, static_cast<std::uint32_t>(c.duration));
    return out;
}

RobotControlCommand decode_command(const std::vector<std::uint8_t>& in) {
    if (in.size() != kPayloadSize) throw std::invalid_argument("invalid command packet length");
    std::size_t at = 0;
    std::uint64_t fingerprint = 0;
    for (int i = 0; i < 8; ++i) fingerprint = (fingerprint << 8) | in[at++];
    if (fingerprint != kFingerprint) throw std::invalid_argument("command fingerprint mismatch");
    RobotControlCommand c{static_cast<std::int8_t>(in[at++]), static_cast<std::int8_t>(in[at++]),
        static_cast<std::int8_t>(in[at++]), static_cast<std::int8_t>(in[at++])};
    get_array(in, at, c.vel_des); get_array(in, at, c.rpy_des); get_array(in, at, c.pos_des);
    get_array(in, at, c.acc_des); get_array(in, at, c.ctrl_point); get_array(in, at, c.foot_pose);
    get_array(in, at, c.step_height);
    c.value = static_cast<std::int32_t>(get_u32(in, at)); c.duration = static_cast<std::int32_t>(get_u32(in, at));
    return c;
}
}  // namespace doogle::protocol
