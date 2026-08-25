#pragma once

#include <opencv2/core.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace doogle::protocol {

inline constexpr std::string_view kRawImageProtocol{"machine-dog-raw-image-v1"};
inline constexpr std::string_view kJpegImageProtocol{"machine-dog-jpeg-image-v1"};

struct ImageHeader {
    std::string protocol;
    std::string payload_format;
    std::string encoding;
    std::uint64_t sequence{};
    std::uint64_t capture_stamp_ns{};
    int width{};
    int height{};
    int step{};
    int jpeg_quality{};
    std::size_t data_length{};
};

struct DecodedImage {
    ImageHeader header;
    cv::Mat frame;
    std::string wire_format;
};

[[nodiscard]] std::optional<ImageHeader> parse_image_header(std::string_view json);
[[nodiscard]] std::optional<DecodedImage> decode_image_packet(
    std::string_view header_json, const std::vector<std::uint8_t>& payload,
    std::string* error = nullptr);

}  // namespace doogle::protocol
