#include <opencv2/imgcodecs.hpp>

#include <zlib.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "instance/runtime/depth_assembler.hpp"
#include "instance/runtime/udp_command_receiver.hpp"
#include "service/protocol/image_packet.hpp"
#include "service/protocol/audio_codec.hpp"
#include "service/protocol/sensor_packets.hpp"
#include "service/protocol/lcm/command_codec.hpp"

namespace {

void put16(std::vector<std::uint8_t>& bytes, std::size_t at, std::uint16_t value) {
    bytes[at] = static_cast<std::uint8_t>(value);
    bytes[at + 1] = static_cast<std::uint8_t>(value >> 8);
}
void put32(std::vector<std::uint8_t>& bytes, std::size_t at, std::uint32_t value) {
    for (int index = 0; index < 4; ++index) bytes[at + index] = static_cast<std::uint8_t>(value >> (index * 8));
}
void put64(std::vector<std::uint8_t>& bytes, std::size_t at, std::uint64_t value) {
    for (int index = 0; index < 8; ++index) bytes[at + index] = static_cast<std::uint8_t>(value >> (index * 8));
}

std::vector<std::uint8_t> depth_packet(std::uint32_t sequence, const std::vector<std::uint8_t>& wire,
                                       std::uint16_t width, std::uint16_t height, std::uint16_t index,
                                       std::uint16_t count, std::size_t begin, std::size_t length,
                                       bool compressed) {
    std::vector<std::uint8_t> packet(40 + length);
    std::memcpy(packet.data(), "DEP1", 4);
    packet[4] = 1;
    packet[5] = compressed ? 1 : 0;
    put16(packet, 6, 40);
    put32(packet, 8, sequence);
    put64(packet, 12, 123456789);
    put16(packet, 20, width);
    put16(packet, 22, height);
    put32(packet, 24, static_cast<std::uint32_t>(wire.size()));
    put16(packet, 28, index);
    put16(packet, 30, count);
    put32(packet, 32, static_cast<std::uint32_t>(begin));
    put16(packet, 36, static_cast<std::uint16_t>(length));
    put16(packet, 38, 1000);
    std::copy_n(wire.begin() + begin, length, packet.begin() + 40);
    return packet;
}

void test_image_packets() {
    const std::string header =
        R"({"protocol":"machine-dog-raw-image-v1","payload_format":"raw","encoding":"rgb8","seq":7,"capture_stamp_ns":9,"width":2,"height":1,"step":6,"data_len":6})";
    const std::vector<std::uint8_t> rgb{255, 0, 0, 0, 255, 0};
    std::string error;
    const auto decoded = doogle::protocol::decode_image_packet(header, rgb, &error);
    assert(decoded.has_value());
    assert(decoded->frame.type() == CV_8UC3);
    assert(decoded->frame.at<cv::Vec3b>(0, 0) == cv::Vec3b(0, 0, 255));

    cv::Mat source(8, 9, CV_8UC3, cv::Scalar{20, 40, 200});
    std::vector<std::uint8_t> jpeg;
    const bool encoded = cv::imencode(".jpg", source, jpeg);
    assert(encoded);
    const auto jpeg_header = std::string{"{\"protocol\":\"machine-dog-jpeg-image-v1\",\"payload_format\":\"jpeg\",\"encoding\":\"bgr8\",\"width\":9,\"height\":8,\"data_len\":"} +
                             std::to_string(jpeg.size()) + "}";
    const auto jpeg_decoded = doogle::protocol::decode_image_packet(jpeg_header, jpeg, &error);
    assert(jpeg_decoded.has_value());
    assert(jpeg_decoded->frame.size() == source.size());
}

void test_depth_reassembly(bool compressed) {
    constexpr std::uint16_t width = 4;
    constexpr std::uint16_t height = 3;
    std::vector<std::uint16_t> values(width * height);
    for (std::size_t index = 0; index < values.size(); ++index) values[index] = static_cast<std::uint16_t>(1000 + index);
    std::vector<std::uint8_t> raw(values.size() * sizeof(std::uint16_t));
    std::memcpy(raw.data(), values.data(), raw.size());
    std::vector<std::uint8_t> wire = raw;
    if (compressed) {
        uLongf size = compressBound(raw.size());
        wire.resize(size);
        const int status = compress2(wire.data(), &size, raw.data(), raw.size(), Z_BEST_SPEED);
        assert(status == Z_OK);
        wire.resize(size);
    }
    const std::size_t chunk = 7;
    const auto count = static_cast<std::uint16_t>((wire.size() + chunk - 1) / chunk);
    std::vector<doogle::protocol::DepthFragment> fragments;
    for (std::uint16_t index = 0; index < count; ++index) {
        const std::size_t begin = index * chunk;
        const std::size_t length = std::min(chunk, wire.size() - begin);
        const auto decoded = doogle::protocol::decode_depth_fragment(
            depth_packet(17, wire, width, height, index, count, begin, length, compressed));
        assert(decoded.has_value());
        fragments.push_back(*decoded);
    }
    std::reverse(fragments.begin(), fragments.end());
    doogle::runtime::DepthAssembler assembler;
    std::optional<doogle::runtime::DepthFrame> frame;
    for (const auto& fragment : fragments) {
        auto result = assembler.accept(fragment);
        if (result) frame = std::move(result);
    }
    assert(frame.has_value());
    assert(frame->sequence == 17);
    assert(frame->depth_mm.rows == height && frame->depth_mm.cols == width);
    for (std::size_t index = 0; index < values.size(); ++index)
        assert(frame->depth_mm.ptr<std::uint16_t>()[index] == values[index]);
}

void test_tof_decode() {
    std::vector<std::uint8_t> packet(528);
    std::memcpy(packet.data(), "TOF1", 4);
    put32(packet, 4, 19);
    put32(packet, 8, 64);
    put32(packet, 12, 64);
    for (int index = 0; index < 128; ++index) {
        const float value = static_cast<float>(index);
        std::uint32_t raw{};
        std::memcpy(&raw, &value, sizeof(raw));
        put32(packet, 16 + index * 4, raw);
    }
    const auto decoded = doogle::protocol::decode_tof_packet(packet);
    assert(decoded.has_value());
    assert(decoded->sequence == 19 && decoded->left_head[63] == 63.0F && decoded->right_head[0] == 64.0F);
}

void test_audio_codec() {
    const auto payload = doogle::protocol::encode_audio_message("set_volume", "{\"value\":40}");
    const auto decoded = doogle::protocol::decode_audio_message(payload);
    assert(decoded.has_value());
    assert(decoded->command == "set_volume" && decoded->data == "{\"value\":40}");
    const auto proxy = doogle::protocol::encode_audio_proxy_frame(27, *decoded);
    const auto proxy_decoded = doogle::protocol::decode_audio_proxy_frame(proxy);
    assert(proxy_decoded.has_value());
    assert(!proxy_decoded->response && proxy_decoded->request_id == 27);
}

std::uint16_t available_udp_port() {
    const int socket = ::socket(AF_INET, SOCK_DGRAM, 0);
    assert(socket >= 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    assert(::bind(socket, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == 0);
    socklen_t size = sizeof(address);
    assert(::getsockname(socket, reinterpret_cast<sockaddr*>(&address), &size) == 0);
    const auto port = ntohs(address.sin_port);
    ::close(socket);
    return port;
}

void test_udp_command_receiver_and_full_command_slot() {
    const auto port = available_udp_port();
    doogle::runtime::UdpCommandReceiver receiver{"127.0.0.1", port};
    assert(receiver.open());
    doogle::protocol::RobotControlCommand sent;
    sent.mode = 62;
    sent.gait_id = 83;
    sent.contact = 15;
    sent.vel_des = {0.11F, -0.07F, 0.03F};
    sent.rpy_des = {0.1F, 0.22F, -0.1F};
    sent.pos_des[2] = -0.12F;
    sent.step_height = {0.035F, 0.035F};
    sent.duration = 900;
    const auto packet = doogle::protocol::encode_command(sent);
    const int socket = ::socket(AF_INET, SOCK_DGRAM, 0);
    assert(socket >= 0);
    sockaddr_in target{};
    target.sin_family = AF_INET;
    target.sin_port = htons(port);
    target.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    assert(::sendto(socket, packet.data(), packet.size(), 0,
                    reinterpret_cast<const sockaddr*>(&target), sizeof(target)) ==
           static_cast<ssize_t>(packet.size()));
    ::close(socket);
    std::optional<doogle::protocol::RobotControlCommand> received;
    for (int attempt = 0; attempt < 20 && !received; ++attempt) {
        received = receiver.poll();
        if (!received) std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    assert(received.has_value());
    assert(received->mode == sent.mode && received->gait_id == sent.gait_id);
    assert(received->vel_des == sent.vel_des);
    assert(received->rpy_des == sent.rpy_des);
    assert(received->step_height == sent.step_height);
    assert(received->duration == sent.duration);
    const auto health = receiver.health();
    assert(health.packets == 1 && health.accepted == 1 && health.decode_errors == 0);
}

}  // namespace

int main() {
    test_image_packets();
    test_depth_reassembly(false);
    test_depth_reassembly(true);
    test_tof_decode();
    test_audio_codec();
    test_udp_command_receiver_and_full_command_slot();
    return 0;
}
