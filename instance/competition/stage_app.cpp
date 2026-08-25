#include "instance/competition/stage_app.hpp"

#include <array>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/dnn.hpp>

#include "instance/competition/business_engine.hpp"
#include "instance/runtime/zmq_frame_receiver.hpp"
#include "instance/runtime/udp_sensor_hub.hpp"
#include "service/adapter/udp_motion_sink.hpp"

namespace doogle::stage6 {
namespace {
constexpr std::array<std::string_view, 6> kStages{
    "stage1", "stage2", "stage3", "stage4", "stage5", "stage6"};

struct FrameSource {
    cv::Mat latest;
    std::unique_ptr<runtime::ZmqFrameReceiver> receiver;
};

bool configure_source(FrameSource& source, const std::string& path, const std::string& endpoint,
                      int image_flags) {
    if (!path.empty()) {
        source.latest = cv::imread(path, image_flags);
        if (source.latest.empty()) return false;
    }
    if (!endpoint.empty()) {
        source.receiver = std::make_unique<runtime::ZmqFrameReceiver>(endpoint);
        if (!source.receiver->open()) return false;
    }
    return true;
}

void refresh(FrameSource& source, std::chrono::milliseconds timeout) {
    if (!source.receiver) return;
    if (auto frame = source.receiver->receive(timeout)) source.latest = std::move(frame->frame);
}
}

int run_stage(std::string_view stage_name) {
    StageRunConfig config;
    config.stage_name = stage_name;
    return run_stage(config);
}

int run_stage(const StageRunConfig& config) {
    const auto stage = competition::parse_stage_id(config.stage_name);
    if (!stage || config.ticks < 1 || config.period.count() < 0) return 2;

    FrameSource rgb, ai, left, right, depth;
    if (!configure_source(rgb, config.rgb_path, config.rgb_endpoint, cv::IMREAD_COLOR) ||
        !configure_source(ai, config.ai_path, config.ai_endpoint, cv::IMREAD_COLOR) ||
        !configure_source(left, config.left_fisheye_path, config.left_fisheye_endpoint, cv::IMREAD_COLOR) ||
        !configure_source(right, config.right_fisheye_path, config.right_fisheye_endpoint, cv::IMREAD_COLOR) ||
        !configure_source(depth, config.depth_path, config.depth_endpoint, cv::IMREAD_ANYDEPTH)) {
        std::cerr << "sensor_source_open_failed\n";
        return 3;
    }

    const auto phase = competition::parse_business_phase(*stage, config.start_phase);
    if (!phase) {
        std::cerr << "invalid_start_phase\n";
        return 2;
    }
    competition::BusinessEngineConfig engine_config;
    engine_config.start_phase[static_cast<std::size_t>(*stage)] = *phase;
    engine_config.nominal_tick = config.period;
    engine_config.stage6_allow_partial_range = config.stage6_allow_partial_range;
    competition::BusinessEngine engine{engine_config};
    if (config.motion_host.empty() != (config.motion_port == 0)) {
        std::cerr << "motion_target_incomplete\n";
        return 2;
    }
    std::unique_ptr<adapters::UdpMotionSink> motion_sink;
    if (!config.motion_host.empty())
        motion_sink = std::make_unique<adapters::UdpMotionSink>(config.motion_host, config.motion_port);
    cv::dnn::Net football_network;
    if (!config.football_model_path.empty()) {
        try {
            football_network = cv::dnn::readNetFromONNX(config.football_model_path);
        } catch (const cv::Exception&) {
            std::cerr << "football_model_open_failed\n";
            return 3;
        }
    }
    runtime::UdpSensorConfig sensor_config;
    sensor_config.bind_host = config.sensor_bind_host;
    sensor_config.pose_port = config.pose_port;
    sensor_config.depth_port = config.depth_port;
    sensor_config.tof_port = config.tof_port;
    runtime::UdpSensorHub sensor_hub{sensor_config};
    if (!sensor_hub.open()) {
        std::cerr << "udp_sensor_open_failed\n";
        return 3;
    }
    for (int tick = 0; tick < config.ticks; ++tick) {
        const auto receive_timeout = tick == 0 ? config.period : std::chrono::milliseconds{1};
        refresh(rgb, receive_timeout);
        refresh(ai, std::chrono::milliseconds{1});
        refresh(left, std::chrono::milliseconds{1});
        refresh(right, std::chrono::milliseconds{1});
        refresh(depth, std::chrono::milliseconds{1});
        sensor_hub.poll();

        competition::SensorFrame frame;
        frame.rgb = rgb.latest;
        frame.ai = ai.latest;
        frame.left_fisheye = left.latest;
        frame.right_fisheye = right.latest;
        frame.depth = depth.latest;
        if (sensor_hub.latest_depth()) frame.depth = sensor_hub.latest_depth()->depth_mm;
        if (sensor_hub.latest_tof()) {
            perception::TofFrame tof;
            tof.left = sensor_hub.latest_tof()->left_head;
            tof.right = sensor_hub.latest_tof()->right_head;
            frame.tof = tof;
        }
        if (sensor_hub.latest_pose()) {
            constexpr double kRadiansToDegrees = 57.29577951308232;
            const auto& pose = *sensor_hub.latest_pose();
            frame.pose = competition::PoseSample{
                pose.xyz[0], pose.xyz[1], pose.rpy[2] * kRadiansToDegrees,
                static_cast<std::uint64_t>(pose.timestamp), sensor_hub.latest_pose_sequence()};
        }
        frame.now = std::chrono::steady_clock::now();
        if (!football_network.empty() && !frame.rgb.empty())
            frame.football = perception::detect_football_dnn(frame.rgb, football_network);
        auto decision = engine.tick(*stage, frame);
        decision.command.life_count = static_cast<std::int8_t>((tick + 1) % 128);
        if (motion_sink) motion_sink->publish(decision.command);
        std::cout << competition::stage_name(decision.stage)
                  << " tick=" << tick
                  << " phase=" << decision.phase
                  << " step=" << decision.step
                  << " phase_key=" << decision.phase_key
                  << " action=" << decision.action
                  << " reason=" << decision.reason
                  << " mode=" << static_cast<int>(decision.command.mode)
                  << " gait=" << static_cast<int>(decision.command.gait_id)
                  << " vx=" << decision.command.vel_des[0]
                  << " vy=" << decision.command.vel_des[1]
                  << " wz=" << decision.command.vel_des[2]
                  << " complete=" << decision.stage_complete
                  << " fail_closed=" << decision.fail_closed
                  << " degraded=" << decision.degraded;
        if (decision.speech) std::cout << " speech=" << *decision.speech;
        std::cout << '\n';
        if (decision.stage_complete && config.stop_on_complete) {
            if (motion_sink) {
                protocol::RobotControlCommand stop;
                stop.mode = 12;
                stop.contact = 15;
                stop.life_count = static_cast<std::int8_t>((tick + 2) % 128);
                motion_sink->publish(stop);
            }
            return 0;
        }
        if (tick + 1 < config.ticks) std::this_thread::sleep_for(config.period);
    }
    if (motion_sink) {
        protocol::RobotControlCommand stop;
        stop.mode = 12;
        stop.contact = 15;
        stop.life_count = static_cast<std::int8_t>((config.ticks + 1) % 128);
        motion_sink->publish(stop);
    }
    return config.allow_incomplete ? 0 : 4;
}

int run_all_stages() {
    StageRunConfig config;
    return run_all_stages(config);
}

int run_all_stages(const StageRunConfig& base) {
    for (const auto stage : kStages) {
        auto config = base;
        config.stage_name = stage;
        const int status = run_stage(config);
        if (status != 0) return status;
    }
    return 0;
}

}  // namespace doogle::stage6
