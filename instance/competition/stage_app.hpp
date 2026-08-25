#pragma once

#include <chrono>
#include <cstdint>
#include <string_view>
#include <string>

namespace doogle::stage6 {

struct StageRunConfig {
    std::string stage_name;
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
    std::string start_phase{"1"};
    std::string sensor_bind_host{"127.0.0.1"};
    std::uint16_t pose_port{};
    std::uint16_t depth_port{};
    std::uint16_t tof_port{};
    std::string motion_host;
    std::uint16_t motion_port{};
    int ticks{1};
    std::chrono::milliseconds period{50};
    bool stop_on_complete{true};
    bool allow_incomplete{true};
    bool stage6_allow_partial_range{};
};

[[nodiscard]] int run_stage(std::string_view stage_name);
[[nodiscard]] int run_stage(const StageRunConfig& config);
[[nodiscard]] int run_all_stages();
[[nodiscard]] int run_all_stages(const StageRunConfig& config);

}  // namespace doogle::stage6
