#include "service/protocol/audio_codec.hpp"

#include <algorithm>

namespace doogle::protocol {
namespace {

void append_u32_be(std::vector<std::uint8_t>& output, std::uint32_t value) {
    for (int shift = 24; shift >= 0; shift -= 8) output.push_back(static_cast<std::uint8_t>(value >> shift));
}
void append_u64_be(std::vector<std::uint8_t>& output, std::uint64_t value) {
    for (int shift = 56; shift >= 0; shift -= 8) output.push_back(static_cast<std::uint8_t>(value >> shift));
}
std::uint32_t u32_be(const std::vector<std::uint8_t>& input, std::size_t offset) {
    std::uint32_t value{};
    for (std::size_t index = 0; index < 4; ++index) value = (value << 8) | input[offset + index];
    return value;
}
std::uint64_t u64_be(const std::vector<std::uint8_t>& input, std::size_t offset) {
    std::uint64_t value{};
    for (std::size_t index = 0; index < 8; ++index) value = (value << 8) | input[offset + index];
    return value;
}
void append_string(std::vector<std::uint8_t>& output, std::string_view value) {
    append_u32_be(output, static_cast<std::uint32_t>(value.size() + 1));
    output.insert(output.end(), value.begin(), value.end());
    output.push_back(0);
}
std::optional<std::string> read_string(const std::vector<std::uint8_t>& input, std::size_t& offset) {
    if (offset + 4 > input.size()) return std::nullopt;
    const auto size = u32_be(input, offset);
    offset += 4;
    if (size == 0 || size > input.size() - offset || input[offset + size - 1] != 0) return std::nullopt;
    std::string value(reinterpret_cast<const char*>(input.data() + offset), size - 1);
    offset += size;
    return value;
}

}  // namespace

std::vector<std::uint8_t> encode_audio_message(std::string_view command, std::string_view data) {
    std::vector<std::uint8_t> output;
    output.reserve(18 + command.size() + data.size());
    append_u64_be(output, kAudioLcmFingerprint);
    append_string(output, command);
    append_string(output, data);
    return output;
}

std::optional<AudioMessage> decode_audio_message(const std::vector<std::uint8_t>& payload) {
    if (payload.size() < 18 || u64_be(payload, 0) != kAudioLcmFingerprint) return std::nullopt;
    std::size_t offset = 8;
    auto command = read_string(payload, offset);
    auto data = read_string(payload, offset);
    if (!command || !data || offset != payload.size()) return std::nullopt;
    return AudioMessage{std::move(*command), std::move(*data)};
}

std::vector<std::uint8_t> encode_audio_proxy_frame(std::uint32_t request_id,
                                                   const AudioMessage& message, bool response) {
    const auto payload = encode_audio_message(message.command, message.data);
    std::vector<std::uint8_t> output;
    output.reserve(12 + payload.size());
    const auto magic = response ? std::string_view{"AUDR"} : std::string_view{"AUDQ"};
    output.insert(output.end(), magic.begin(), magic.end());
    append_u32_be(output, request_id);
    append_u32_be(output, static_cast<std::uint32_t>(payload.size()));
    output.insert(output.end(), payload.begin(), payload.end());
    return output;
}

std::optional<AudioProxyFrame> decode_audio_proxy_frame(const std::vector<std::uint8_t>& packet) {
    if (packet.size() < 12) return std::nullopt;
    const bool response = std::equal(packet.begin(), packet.begin() + 4, "AUDR");
    const bool request = std::equal(packet.begin(), packet.begin() + 4, "AUDQ");
    if (!request && !response) return std::nullopt;
    const auto size = u32_be(packet, 8);
    if (size != packet.size() - 12) return std::nullopt;
    const std::vector<std::uint8_t> payload(packet.begin() + 12, packet.end());
    auto message = decode_audio_message(payload);
    if (!message) return std::nullopt;
    return AudioProxyFrame{response, u32_be(packet, 4), std::move(*message)};
}

}  // namespace doogle::protocol
