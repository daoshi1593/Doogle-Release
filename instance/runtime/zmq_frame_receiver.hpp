#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "service/protocol/image_packet.hpp"

namespace doogle::runtime {

struct FrameReceiverHealth {
    std::string endpoint;
    std::uint64_t received_total{};
    std::uint64_t decode_errors{};
    std::uint64_t dropped_sequence{};
    std::uint64_t last_sequence{};
    double receive_fps{};
};

class ZmqFrameReceiver {
public:
    explicit ZmqFrameReceiver(std::string endpoint);
    ~ZmqFrameReceiver();
    ZmqFrameReceiver(const ZmqFrameReceiver&) = delete;
    ZmqFrameReceiver& operator=(const ZmqFrameReceiver&) = delete;

    [[nodiscard]] bool open();
    void close();
    [[nodiscard]] std::optional<protocol::DecodedImage> receive(std::chrono::milliseconds timeout);
    [[nodiscard]] FrameReceiverHealth health() const;

private:
    std::string endpoint_;
    void* context_{};
    void* socket_{};
    std::uint64_t received_total_{};
    std::uint64_t decode_errors_{};
    std::uint64_t dropped_sequence_{};
    std::uint64_t last_sequence_{};
    std::chrono::steady_clock::time_point window_start_{std::chrono::steady_clock::now()};
};

}  // namespace doogle::runtime
