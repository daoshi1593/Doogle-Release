#include <iostream>
#include <limits>
#include <string>

#include "instance/competition/stage_app.hpp"

namespace {
bool integer_value(const char* value, int& output) {
    try {
        std::size_t end{};
        const long parsed = std::stol(value, &end);
        if (value[end] != '\0' || parsed < 0 || parsed > std::numeric_limits<int>::max()) return false;
        output = static_cast<int>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}
bool port_value(const char* value, std::uint16_t& output) {
    int parsed{};
    if (!integer_value(value, parsed) || parsed < 1 || parsed > 65535) return false;
    output = static_cast<std::uint16_t>(parsed);
    return true;
}
}

int main(int argc, char** argv) {
    if (argc == 1) return doogle::stage6::run_all_stages();
    doogle::stage6::StageRunConfig config;
    bool run_all{};
    for (int index = 1; index < argc; ++index) {
        const std::string key = argv[index];
        auto value = [&]() -> const char* {
            if (index + 1 >= argc) return nullptr;
            return argv[++index];
        };
        if (key == "--stage") { const auto* v = value(); if (!v) return 2; config.stage_name = v; }
        else if (key == "--all") run_all = true;
        else if (key == "--start-phase") { const auto* v = value(); if (!v) return 2; config.start_phase = v; }
        else if (key == "--rgb") { const auto* v = value(); if (!v) return 2; config.rgb_path = v; }
        else if (key == "--ai") { const auto* v = value(); if (!v) return 2; config.ai_path = v; }
        else if (key == "--left") { const auto* v = value(); if (!v) return 2; config.left_fisheye_path = v; }
        else if (key == "--right") { const auto* v = value(); if (!v) return 2; config.right_fisheye_path = v; }
        else if (key == "--depth") { const auto* v = value(); if (!v) return 2; config.depth_path = v; }
        else if (key == "--rgb-endpoint") { const auto* v = value(); if (!v) return 2; config.rgb_endpoint = v; }
        else if (key == "--ai-endpoint") { const auto* v = value(); if (!v) return 2; config.ai_endpoint = v; }
        else if (key == "--left-endpoint") { const auto* v = value(); if (!v) return 2; config.left_fisheye_endpoint = v; }
        else if (key == "--right-endpoint") { const auto* v = value(); if (!v) return 2; config.right_fisheye_endpoint = v; }
        else if (key == "--depth-endpoint") { const auto* v = value(); if (!v) return 2; config.depth_endpoint = v; }
        else if (key == "--football-model") { const auto* v = value(); if (!v) return 2; config.football_model_path = v; }
        else if (key == "--sensor-bind") { const auto* v = value(); if (!v) return 2; config.sensor_bind_host = v; }
        else if (key == "--pose-port") { const auto* v = value(); if (!v || !port_value(v, config.pose_port)) return 2; }
        else if (key == "--depth-port") { const auto* v = value(); if (!v || !port_value(v, config.depth_port)) return 2; }
        else if (key == "--tof-port") { const auto* v = value(); if (!v || !port_value(v, config.tof_port)) return 2; }
        else if (key == "--motion-host") { const auto* v = value(); if (!v) return 2; config.motion_host = v; }
        else if (key == "--motion-port") { const auto* v = value(); if (!v || !port_value(v, config.motion_port)) return 2; }
        else if (key == "--ticks") { const auto* v = value(); if (!v || !integer_value(v, config.ticks)) return 2; }
        else if (key == "--period-ms") {
            const auto* v = value(); int milliseconds{};
            if (!v || !integer_value(v, milliseconds)) return 2;
            config.period = std::chrono::milliseconds{milliseconds};
        } else if (key == "--keep-running-after-complete") config.stop_on_complete = false;
        else if (key == "--require-complete") config.allow_incomplete = false;
        else if (key == "--allow-partial-range") config.stage6_allow_partial_range = true;
        else {
            std::cerr << "unknown_option=" << key << '\n';
            return 2;
        }
    }
    if (run_all) {
        if (!config.stage_name.empty() || config.start_phase != "1") return 2;
        config.allow_incomplete = false;
        return doogle::stage6::run_all_stages(config);
    }
    if (config.stage_name.empty()) return 2;
    return doogle::stage6::run_stage(config);
}
