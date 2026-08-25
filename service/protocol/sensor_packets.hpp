#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace doogle::protocol {

struct DepthFragment {
    std::uint32_t sequence{};
    std::uint64_t capture_stamp_ns{};
    std::uint16_t width{};
    std::uint16_t height{};
    std::uint32_t frame_bytes{};
    std::uint16_t fragment_index{};
    std::uint16_t fragment_count{};
    std::uint32_t offset{};
    std::uint16_t payload_bytes{};
    std::uint16_t unit_micrometers{};
    bool compressed{false};
    std::vector<std::uint8_t> payload;
};

struct TofPacket {
    std::uint32_t sequence{};
    std::array<float, 64> left_head{};
    std::array<float, 64> right_head{};
};

[[nodiscard]] std::optional<DepthFragment> decode_depth_fragment(const std::vector<std::uint8_t>&);
[[nodiscard]] std::optional<TofPacket> decode_tof_packet(const std::vector<std::uint8_t>&);
[[nodiscard]] bool valid_tof_packet(const std::vector<std::uint8_t>&);

}  // namespace doogle::protocol
