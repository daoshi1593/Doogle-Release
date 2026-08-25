#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "instance/api/sensor_provider.hpp"

namespace doogle::runtime {

struct StandardSensorProviderConfig {
    std::string rgb_path;
    std::string ai_path;
    std::string left_fisheye_path;
    std::string right_fisheye_path;
    std::string depth_path;
    std::string rgb_endpoint;
    std::string ai_endpoint;
    std::string left_fisheye_endpoint;
    std::string right_fisheye_endpoint;
    std::string depth_endpoint;
    std::string football_model_path;
    std::string sensor_bind_host{"127.0.0.1"};
    std::uint16_t pose_port{};
    std::uint16_t depth_port{};
    std::uint16_t tof_port{};
    bool allow_empty_frames{};
};

class StandardSensorProvider final : public instance::ISensorProvider {
public:
    explicit StandardSensorProvider(StandardSensorProviderConfig config = {});
    ~StandardSensorProvider() override;
    StandardSensorProvider(const StandardSensorProvider&) = delete;
    StandardSensorProvider& operator=(const StandardSensorProvider&) = delete;
    StandardSensorProvider(StandardSensorProvider&&) noexcept;
    StandardSensorProvider& operator=(StandardSensorProvider&&) noexcept;

    [[nodiscard]] bool open() override;
    void close() noexcept override;
    [[nodiscard]] instance::SensorRead read(std::chrono::milliseconds timeout) override;

    [[nodiscard]] const StandardSensorProviderConfig& config() const noexcept { return config_; }

private:
    struct Impl;

    StandardSensorProviderConfig config_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace doogle::runtime
