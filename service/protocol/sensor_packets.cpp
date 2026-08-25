#include "service/protocol/sensor_packets.hpp"

#include <bit>
#include <cmath>
#include <cstring>

namespace doogle::protocol {
namespace {
std::uint16_t u16(const std::vector<std::uint8_t>& b, std::size_t at) {
    return static_cast<std::uint16_t>(std::uint16_t(b[at]) |
                                      static_cast<std::uint16_t>(std::uint16_t(b[at + 1]) << 8));
}
std::uint32_t u32(const std::vector<std::uint8_t>& b, std::size_t at) {
    return std::uint32_t(b[at]) | (std::uint32_t(b[at + 1]) << 8) |
           (std::uint32_t(b[at + 2]) << 16) | (std::uint32_t(b[at + 3]) << 24);
}
std::uint64_t u64(const std::vector<std::uint8_t>& b, std::size_t at) {
    std::uint64_t value = 0;
    for (std::size_t i = 0; i < 8; ++i) value |= std::uint64_t(b[at + i]) << (8 * i);
    return value;
}
float f32(const std::vector<std::uint8_t>& b, std::size_t at) {
    const auto raw = u32(b, at);
    return std::bit_cast<float>(raw);
}
}  // namespace

std::optional<DepthFragment> decode_depth_fragment(const std::vector<std::uint8_t>& packet) {
    if (packet.size() < 40 || packet[0] != 'D' || packet[1] != 'E' || packet[2] != 'P' || packet[3] != '1') {
        return std::nullopt;
    }
    if (packet[4] != 1 || u16(packet, 6) != 40) return std::nullopt;
    DepthFragment result;
    if ((packet[5] & 0xFEU) != 0) return std::nullopt;
    result.compressed = (packet[5] & 1U) != 0;
    result.sequence = u32(packet, 8);
    result.capture_stamp_ns = u64(packet, 12);
    result.width = u16(packet, 20);
    result.height = u16(packet, 22);
    result.frame_bytes = u32(packet, 24);
    result.fragment_index = u16(packet, 28);
    result.fragment_count = u16(packet, 30);
    result.offset = u32(packet, 32);
    result.payload_bytes = u16(packet, 36);
    result.unit_micrometers = u16(packet, 38);
    if (result.fragment_count == 0 || result.fragment_index >= result.fragment_count ||
        result.payload_bytes != packet.size() - 40 || result.offset + result.payload_bytes > result.frame_bytes ||
        result.width == 0 || result.height == 0 || result.frame_bytes == 0 ||
        result.frame_bytes > 16U * 1024U * 1024U) {
        return std::nullopt;
    }
    result.payload.assign(packet.begin() + 40, packet.end());
    return result;
}

bool valid_tof_packet(const std::vector<std::uint8_t>& packet) {
    return decode_tof_packet(packet).has_value();
}

std::optional<TofPacket> decode_tof_packet(const std::vector<std::uint8_t>& packet) {
    constexpr std::size_t kPacketSize = 4 + 3 * sizeof(std::uint32_t) + 128 * sizeof(float);
    if (packet.size() != kPacketSize || packet[0] != 'T' || packet[1] != 'O' ||
        packet[2] != 'F' || packet[3] != '1' || u32(packet, 8) != 64 || u32(packet, 12) != 64) {
        return std::nullopt;
    }
    TofPacket result;
    result.sequence = u32(packet, 4);
    std::size_t offset = 16;
    for (auto& value : result.left_head) {
        value = f32(packet, offset);
        offset += sizeof(float);
        if (!std::isfinite(value)) return std::nullopt;
    }
    for (auto& value : result.right_head) {
        value = f32(packet, offset);
        offset += sizeof(float);
        if (!std::isfinite(value)) return std::nullopt;
    }
    return result;
}

}  // namespace doogle::protocol
