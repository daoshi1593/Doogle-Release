#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "instance/runtime/depth_assembler.hpp"
#include "service/protocol/pose_protocol.hpp"
#include "service/protocol/sensor_packets.hpp"

namespace doogle::runtime {

struct UdpSensorConfig {
    std::string bind_host{"127.0.0.1"};
    std::uint16_t pose_port{};
    std::uint16_t depth_port{};
    std::uint16_t tof_port{};
    std::string pose_channel{"global_to_robot"};
};

struct UdpSensorHealth {
    std::uint64_t pose_packets{};
    std::uint64_t depth_packets{};
    std::uint64_t tof_packets{};
    std::uint64_t decode_errors{};
};

class UdpSensorHub {
public:
    explicit UdpSensorHub(UdpSensorConfig config);
    ~UdpSensorHub();
    UdpSensorHub(const UdpSensorHub&) = delete;
    UdpSensorHub& operator=(const UdpSensorHub&) = delete;

    [[nodiscard]] bool open();
    void close();
    void poll();

    [[nodiscard]] const std::optional<protocol::Pose>& latest_pose() const { return pose_; }
    [[nodiscard]] std::uint32_t latest_pose_sequence() const { return pose_sequence_; }
    [[nodiscard]] const std::optional<DepthFrame>& latest_depth() const { return depth_; }
    [[nodiscard]] const std::optional<protocol::TofPacket>& latest_tof() const { return tof_; }
    [[nodiscard]] UdpSensorHealth health() const { return health_; }

private:
    [[nodiscard]] int open_socket(std::uint16_t port) const;
    void drain_pose();
    void drain_depth();
    void drain_tof();

    UdpSensorConfig config_;
    int pose_socket_{-1};
    int depth_socket_{-1};
    int tof_socket_{-1};
    DepthAssembler depth_assembler_;
    std::optional<protocol::Pose> pose_;
    std::uint32_t pose_sequence_{};
    std::int64_t pose_timestamp_{};
    std::optional<DepthFrame> depth_;
    std::optional<protocol::TofPacket> tof_;
    UdpSensorHealth health_{};
};

}  // namespace doogle::runtime
