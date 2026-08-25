#pragma once

#include <opencv2/core.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include "service/protocol/sensor_packets.hpp"

namespace doogle::runtime {

struct DepthFrame {
    std::uint32_t sequence{};
    std::uint64_t capture_stamp_ns{};
    std::uint16_t unit_micrometers{};
    cv::Mat depth_mm;
};

class DepthAssembler {
public:
    explicit DepthAssembler(std::size_t max_inflight = 4);
    [[nodiscard]] std::optional<DepthFrame> accept(const protocol::DepthFragment& fragment);
    void clear();
    [[nodiscard]] std::uint64_t discarded_frames() const { return discarded_frames_; }

private:
    struct Assembly {
        std::uint16_t width{};
        std::uint16_t height{};
        std::uint16_t unit_micrometers{};
        std::uint16_t fragment_count{};
        std::uint64_t capture_stamp_ns{};
        bool compressed{};
        std::vector<std::uint8_t> payload;
        std::vector<bool> received;
        std::vector<std::pair<std::uint32_t, std::uint32_t>> ranges;
        std::size_t received_count{};
    };

    std::size_t max_inflight_;
    std::unordered_map<std::uint32_t, Assembly> assemblies_;
    std::vector<std::uint32_t> insertion_order_;
    std::uint64_t discarded_frames_{};
};

}  // namespace doogle::runtime
