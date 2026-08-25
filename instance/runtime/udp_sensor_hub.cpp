#include "instance/runtime/udp_sensor_hub.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace doogle::runtime {
namespace {

std::optional<std::vector<std::uint8_t>> receive_packet(int socket, std::size_t capacity) {
    std::vector<std::uint8_t> packet(capacity);
    const auto size = ::recvfrom(socket, packet.data(), packet.size(), MSG_DONTWAIT, nullptr, nullptr);
    if (size < 0) return std::nullopt;
    packet.resize(static_cast<std::size_t>(size));
    return packet;
}

}  // namespace

UdpSensorHub::UdpSensorHub(UdpSensorConfig config) : config_(std::move(config)) {}
UdpSensorHub::~UdpSensorHub() { close(); }

int UdpSensorHub::open_socket(std::uint16_t port) const {
    if (port == 0) return -1;
    const int socket = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket < 0) return -1;
    int reuse = 1;
    ::setsockopt(socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (::inet_pton(AF_INET, config_.bind_host.c_str(), &address.sin_addr) != 1 ||
        ::bind(socket, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) {
        ::close(socket);
        return -1;
    }
    const int flags = ::fcntl(socket, F_GETFL, 0);
    if (flags >= 0) ::fcntl(socket, F_SETFL, flags | O_NONBLOCK);
    return socket;
}

bool UdpSensorHub::open() {
    close();
    pose_socket_ = open_socket(config_.pose_port);
    depth_socket_ = open_socket(config_.depth_port);
    tof_socket_ = open_socket(config_.tof_port);
    const bool pose_ok = config_.pose_port == 0 || pose_socket_ >= 0;
    const bool depth_ok = config_.depth_port == 0 || depth_socket_ >= 0;
    const bool tof_ok = config_.tof_port == 0 || tof_socket_ >= 0;
    if (!pose_ok || !depth_ok || !tof_ok) {
        close();
        return false;
    }
    return true;
}

void UdpSensorHub::close() {
    for (int* socket : std::array<int*, 3>{&pose_socket_, &depth_socket_, &tof_socket_}) {
        if (*socket >= 0) ::close(*socket);
        *socket = -1;
    }
    depth_assembler_.clear();
}

void UdpSensorHub::poll() {
    drain_pose();
    drain_depth();
    drain_tof();
}

void UdpSensorHub::drain_pose() {
    if (pose_socket_ < 0) return;
    while (auto packet = receive_packet(pose_socket_, 64U * 1024U)) {
        ++health_.pose_packets;
        const auto frame = protocol::parse_lc02_frame(*packet, config_.pose_channel);
        const auto pose = frame ? protocol::decode_pose_payload(frame->payload) : std::nullopt;
        const bool finite = pose &&
            std::all_of(pose->xyz.begin(), pose->xyz.end(), [](float value) { return std::isfinite(value); }) &&
            std::all_of(pose->rpy.begin(), pose->rpy.end(), [](float value) { return std::isfinite(value); });
        const bool monotonic = pose && (pose_timestamp_ == 0 || pose->timestamp >= pose_timestamp_);
        if (finite && monotonic) {
            pose_ = *pose;
            pose_sequence_ = frame->sequence;
            pose_timestamp_ = pose->timestamp;
        } else {
            ++health_.decode_errors;
        }
    }
}

void UdpSensorHub::drain_depth() {
    if (depth_socket_ < 0) return;
    while (auto packet = receive_packet(depth_socket_, 65535)) {
        ++health_.depth_packets;
        const auto fragment = protocol::decode_depth_fragment(*packet);
        if (!fragment) {
            ++health_.decode_errors;
            continue;
        }
        if (auto completed = depth_assembler_.accept(*fragment)) depth_ = std::move(completed);
    }
}

void UdpSensorHub::drain_tof() {
    if (tof_socket_ < 0) return;
    while (auto packet = receive_packet(tof_socket_, 2048)) {
        ++health_.tof_packets;
        const auto decoded = protocol::decode_tof_packet(*packet);
        if (decoded) tof_ = *decoded;
        else ++health_.decode_errors;
    }
}

}  // namespace doogle::runtime
