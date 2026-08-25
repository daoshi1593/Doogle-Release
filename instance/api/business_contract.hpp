#pragma once

#include <opencv2/core.hpp>

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "service/perception/depth_tof.hpp"
#include "service/perception/vision.hpp"
#include "service/protocol/robot_control_cmd.hpp"

namespace doogle::competition {

enum class StageId { Stage1, Stage2, Stage3, Stage4, Stage5, Stage6 };

struct PoseSample {
    double x{};
    double y{};
    double yaw_deg{};
    std::uint64_t source_timestamp{};
    std::uint32_t sequence{};
};

struct BusinessObservations {
    std::optional<perception::ColorDetection> red_ball;
    std::optional<perception::ColorDetection> blue_ball;
    std::optional<perception::ColorDetection> blue_cube;
    std::optional<perception::ColorDetection> cola;
    std::optional<perception::ColorDetection> orange_ball;
    std::optional<perception::YellowLineObservation> left_yellow_line;
    std::optional<perception::YellowLineObservation> right_yellow_line;
    std::optional<perception::LaneObservation> rgb_lane;
    std::optional<perception::MarkerObservation> left_bottom_yellow;
    std::optional<perception::MarkerObservation> right_bottom_yellow;
    std::optional<perception::MarkerObservation> top_red;
    std::optional<perception::LimitBarObservation> limit_bar;
    std::optional<bool> yellow_finish_boundary;
    std::optional<bool> dotted_line_left;
    std::optional<bool> dotted_line_right;
};

struct SensorFrame {
    cv::Mat rgb;
    cv::Mat ai;
    cv::Mat left_fisheye;
    cv::Mat right_fisheye;
    cv::Mat depth;
    std::optional<perception::TofFrame> tof;
    std::optional<perception::FootballObservation> football;
    std::optional<PoseSample> pose;
    BusinessObservations observations;
    bool control_safe{true};
    bool stop_requested{};
    std::chrono::steady_clock::time_point now{std::chrono::steady_clock::now()};
};

struct BusinessDecision {
    StageId stage{StageId::Stage1};
    int phase{1};
    int step{};
    protocol::RobotControlCommand command{};
    bool stage_complete{};
    bool fail_closed{};
    bool degraded{};
    perception::VisualTarget target{perception::VisualTarget::None};
    std::string phase_key{"startup_ready"};
    std::string action{"stand"};
    std::string reason{"waiting_for_sensor"};
    std::optional<std::string> speech;
};

}  // namespace doogle::competition
