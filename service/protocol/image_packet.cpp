#include "service/protocol/image_packet.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <charconv>
#include <cctype>

namespace doogle::protocol {
namespace {

std::optional<std::string> string_value(std::string_view json, std::string_view key) {
    const std::string quoted_key = "\"" + std::string(key) + "\"";
    auto position = json.find(quoted_key);
    if (position == std::string_view::npos) return std::nullopt;
    position = json.find(':', position + quoted_key.size());
    if (position == std::string_view::npos) return std::nullopt;
    position = json.find('"', position + 1);
    if (position == std::string_view::npos) return std::nullopt;
    const auto end = json.find('"', position + 1);
    if (end == std::string_view::npos) return std::nullopt;
    return std::string(json.substr(position + 1, end - position - 1));
}

template <typename Number>
std::optional<Number> number_value(std::string_view json, std::string_view key) {
    const std::string quoted_key = "\"" + std::string(key) + "\"";
    auto position = json.find(quoted_key);
    if (position == std::string_view::npos) return std::nullopt;
    position = json.find(':', position + quoted_key.size());
    if (position == std::string_view::npos) return std::nullopt;
    ++position;
    while (position < json.size() && std::isspace(static_cast<unsigned char>(json[position]))) ++position;
    auto end = position;
    while (end < json.size() && (std::isdigit(static_cast<unsigned char>(json[end])) || json[end] == '-')) ++end;
    Number output{};
    const auto result = std::from_chars(json.data() + position, json.data() + end, output);
    if (result.ec != std::errc{}) return std::nullopt;
    return output;
}

std::string normalize_encoding(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
        return character == '-' ? '_' : static_cast<char>(std::tolower(character));
    });
    return value;
}

void set_error(std::string* output, std::string value) {
    if (output) *output = std::move(value);
}

}  // namespace

std::optional<ImageHeader> parse_image_header(std::string_view json) {
    ImageHeader header;
    header.protocol = string_value(json, "protocol").value_or("");
    header.payload_format = string_value(json, "payload_format").value_or("");
    header.encoding = normalize_encoding(string_value(json, "encoding").value_or(""));
    header.sequence = number_value<std::uint64_t>(json, "seq").value_or(0);
    header.capture_stamp_ns = number_value<std::uint64_t>(json, "capture_stamp_ns").value_or(0);
    header.width = number_value<int>(json, "width").value_or(0);
    header.height = number_value<int>(json, "height").value_or(0);
    header.step = number_value<int>(json, "step").value_or(0);
    header.jpeg_quality = number_value<int>(json, "jpeg_quality").value_or(0);
    header.data_length = number_value<std::size_t>(json, "data_len").value_or(0);
    if (header.protocol != kRawImageProtocol && header.protocol != kJpegImageProtocol) return std::nullopt;
    if (header.width <= 0 || header.height <= 0 || header.width > 16384 || header.height > 16384 ||
        header.step < 0 || header.step > 256 * 1024 * 1024) return std::nullopt;
    return header;
}

std::optional<DecodedImage> decode_image_packet(std::string_view header_json,
                                                const std::vector<std::uint8_t>& payload,
                                                std::string* error) {
    const auto parsed = parse_image_header(header_json);
    if (!parsed) {
        set_error(error, "invalid_image_header");
        return std::nullopt;
    }
    auto header = *parsed;
    if (header.data_length != 0 && header.data_length != payload.size()) {
        set_error(error, "payload_length_mismatch");
        return std::nullopt;
    }
    if (header.protocol == kJpegImageProtocol) {
        cv::Mat encoded(1, static_cast<int>(payload.size()), CV_8UC1,
                        const_cast<std::uint8_t*>(payload.data()));
        auto frame = cv::imdecode(encoded, cv::IMREAD_COLOR);
        if (frame.empty()) {
            set_error(error, "jpeg_decode_failed");
            return std::nullopt;
        }
        if (frame.cols != header.width || frame.rows != header.height) {
            set_error(error, "jpeg_dimensions_mismatch");
            return std::nullopt;
        }
        return DecodedImage{std::move(header), std::move(frame), "jpeg"};
    }

    const auto required_bytes = static_cast<std::size_t>(header.height) * static_cast<std::size_t>(header.step);
    if (header.step <= 0 || payload.size() < required_bytes) {
        set_error(error, "raw_payload_too_short");
        return std::nullopt;
    }
    const auto* bytes = payload.data();
    const auto encoding = header.encoding;
    cv::Mat output;
    if (encoding == "bgr8" || encoding == "8uc3") {
        if (header.step < header.width * 3) return std::nullopt;
        cv::Mat wrapped(header.height, header.width, CV_8UC3, const_cast<std::uint8_t*>(bytes),
                        static_cast<std::size_t>(header.step));
        output = wrapped.clone();
    } else if (encoding == "rgb8") {
        if (header.step < header.width * 3) return std::nullopt;
        cv::Mat wrapped(header.height, header.width, CV_8UC3, const_cast<std::uint8_t*>(bytes),
                        static_cast<std::size_t>(header.step));
        cv::cvtColor(wrapped, output, cv::COLOR_RGB2BGR);
    } else if (encoding == "bgra8" || encoding == "rgba8") {
        if (header.step < header.width * 4) return std::nullopt;
        cv::Mat wrapped(header.height, header.width, CV_8UC4, const_cast<std::uint8_t*>(bytes),
                        static_cast<std::size_t>(header.step));
        cv::cvtColor(wrapped, output, encoding == "bgra8" ? cv::COLOR_BGRA2BGR : cv::COLOR_RGBA2BGR);
    } else if (encoding == "mono8" || encoding == "8uc1") {
        if (header.step < header.width) return std::nullopt;
        cv::Mat wrapped(header.height, header.width, CV_8UC1, const_cast<std::uint8_t*>(bytes),
                        static_cast<std::size_t>(header.step));
        cv::cvtColor(wrapped, output, cv::COLOR_GRAY2BGR);
    } else if (encoding == "mono16" || encoding == "16uc1") {
        if (header.step < header.width * 2) return std::nullopt;
        cv::Mat wrapped(header.height, header.width, CV_16UC1, const_cast<std::uint8_t*>(bytes),
                        static_cast<std::size_t>(header.step));
        output = wrapped.clone();
    } else if (encoding == "yuyv" || encoding == "yuyv422" || encoding == "yuv422" ||
               encoding == "yuv422_yuy2") {
        if (header.width % 2 != 0 || header.step < header.width * 2) return std::nullopt;
        cv::Mat wrapped(header.height, header.width, CV_8UC2, const_cast<std::uint8_t*>(bytes),
                        static_cast<std::size_t>(header.step));
        cv::cvtColor(wrapped, output, cv::COLOR_YUV2BGR_YUY2);
    } else if (encoding == "uyvy" || encoding == "uyvy422") {
        if (header.width % 2 != 0 || header.step < header.width * 2) return std::nullopt;
        cv::Mat wrapped(header.height, header.width, CV_8UC2, const_cast<std::uint8_t*>(bytes),
                        static_cast<std::size_t>(header.step));
        cv::cvtColor(wrapped, output, cv::COLOR_YUV2BGR_UYVY);
    } else {
        set_error(error, "unsupported_raw_encoding");
        return std::nullopt;
    }
    return DecodedImage{std::move(header), std::move(output), "raw"};
}

}  // namespace doogle::protocol
