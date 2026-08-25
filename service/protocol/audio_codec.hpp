#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace doogle::protocol {

inline constexpr std::uint64_t kAudioLcmFingerprint = 0xE1EBF77F9E5FB304ULL;

struct AudioMessage {
    std::string command;
    std::string data;
};

struct AudioProxyFrame {
    bool response{};
    std::uint32_t request_id{};
    AudioMessage message;
};

[[nodiscard]] std::vector<std::uint8_t> encode_audio_message(std::string_view command,
                                                            std::string_view data = {});
[[nodiscard]] std::optional<AudioMessage> decode_audio_message(const std::vector<std::uint8_t>& payload);
[[nodiscard]] std::vector<std::uint8_t> encode_audio_proxy_frame(std::uint32_t request_id,
                                                                const AudioMessage& message,
                                                                bool response = false);
[[nodiscard]] std::optional<AudioProxyFrame> decode_audio_proxy_frame(
    const std::vector<std::uint8_t>& packet);

}  // namespace doogle::protocol
