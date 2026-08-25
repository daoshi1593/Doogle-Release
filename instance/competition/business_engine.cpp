#include "instance/competition/business_engine.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>

namespace doogle::competition {
namespace {

constexpr std::array<std::string_view, 6> kStageNames{
    "stage1", "stage2", "stage3", "stage4", "stage5", "stage6"};

const std::vector<BusinessPhaseDefinition> kStage1Phases{
    {"startup_ready", "start cameras and pass control readiness"},
    {"stone_road", "cross the stone road with pose projection"},
    {"stone_road_end_stand", "recover stand after stone road"},
    {"right_tangent_align", "align right fisheye yellow tangent"},
    {"relative_yaw_turn", "shift to yellow ROI and turn relative 90 degrees"},
    {"post_turn_right_tangent_align", "repeat tangent and lateral alignment"},
    {"post_turn_forward", "pose-closed-loop exit until left yellow ROI"},
};

const std::vector<BusinessPhaseDefinition> kStage2Phases{
    {"startup_stand_reference", "stand and capture initial pose reference"},
    {"dual_fisheye_correction", "strict dual-fisheye centering correction"},
    {"start_left_shift", "move left into the recognition route"},
    {"forward_recognition", "four forward centered-ball recognition cycles"},
    {"final_exit", "forward, left turn, forward and right turn"},
    {"backward_recognition", "four backward centered-ball recognition cycles"},
    {"post_backward_no_shift_recognition", "four forward cycles without red shift"},
    {"left_yellow_roi_exit", "stop on left yellow ROI and leave"},
};

const std::vector<BusinessPhaseDefinition> kStage3Phases{
    {"startup_stand", "recover stand"},
    {"fisheye_ready", "wait for both fisheye cameras"},
    {"s_curve_closed_loop", "follow the S curve with stereo yellow tangents"},
    {"right_horizontal_align", "align the right yellow line horizontally"},
};

const std::vector<BusinessPhaseDefinition> kStage4Phases{
    {"config_logs", "initialize business configuration"},
    {"vision_test", "optional vision contract"},
    {"dry_or_speech_test", "optional dry-run and speech contract"},
    {"lcm_prepare_pose", "stand and wait for pose"},
    {"blue_after_bypass_debug", "blue obstacle recovery entry"},
    {"limit_height_football_impact_once_test", "single football impact contract"},
    {"limit_height_left_fisheye_align_test", "post-bar fisheye correction contract"},
    {"entry_move", "move from entry with pose feedback"},
    {"obstacle_handling", "execute three channel obstacle flow"},
    {"entry_vision_scan", "scan remaining targets and settle"},
    {"after_channels_final_tail_debug", "after-channel final tail"},
    {"final_rgb_lane_forward_test", "RGB three-line final lane"},
    {"limit_height_lane_head_center_test", "limit-bar pass-return sequence"},
    {"channel_1_p0_test", "resume from channel one p0"},
    {"channel_2_p0_test", "resume from channel two p0"},
    {"channel_3_p0_test", "resume from channel three p0"},
};

const std::vector<BusinessPhaseDefinition> kStage5Phases{
    {"startup_stand_height", "stand, set body height and wait for cameras"},
    {"startup_straight", "initial straight ROI stop and optional left turn"},
    {"first_closed_loop", "first return-shape closed loop"},
    {"first_post_walk_tail", "first closed-loop tail"},
    {"second_closed_loop", "second return-shape closed loop"},
    {"second_post_walk_tail", "second closed-loop tail"},
    {"final_straight_walk", "third closed-loop straight"},
    {"final_end_turn", "turn after third segment"},
    {"final_end_roi_forward", "fourth ROI closed-loop forward"},
    {"final_after_forward_turn_jump", "final turn and jump"},
};

const std::vector<BusinessPhaseDefinition> kStage6Phases{
    {"startup_sensors", "stand and verify RGB, pose and range sensors"},
    {"search_align_ball", "lateral search and RGB alignment"},
    {"approach_transition", "fixed pose-closed-loop approach and turn"},
    {"push_until_finish", "RGB and ToF fused push until finish boundary"},
    {"finish_kick", "retreat, turn, shift and ToF-adjusted kick"},
};

const std::vector<std::string_view> kStage4BusinessNodes{
    "stage4.prepare_lcm_stand",
    "stage4.debug.depth_probe",
    "stage4.debug.rgb_depth_probe",
    "stage4.debug.stand_rgb_depth_probe",
    "stage4.entry.open_loop_targets",
    "stage4.entry.pose_feedback_targets",
    "stage4.obstacle.entry_center",
    "stage4.obstacle.box_height_sample",
    "stage4.limit_height.pre_approach_center",
    "stage4.limit_height.lane_center",
    "stage4.limit_height.head_yaw",
    "stage4.limit_height.lane_head_center",
    "stage4.limit_height.left_fisheye_tangent_yaw",
    "stage4.limit_height.left_fisheye_roi_lateral",
    "stage4.limit_height.left_fisheye_post_pass_align",
    "stage4.limit_height.prebar_football_lock",
    "stage4.limit_height.p2_initial_axis_trigger",
    "stage4.target.football_lateral_right_impact",
    "stage4.target.p2_sequence_action",
    "stage4.limit_height.second_layer_event",
    "stage4.limit_height.return_yellow_center",
    "stage4.limit_height.channel",
    "stage4.channels.transition_between_channel",
    "stage4.blue_obstacle.transition_dotted_scan",
    "stage4.blue_obstacle.resolve_dotted_side",
    "stage4.blue_obstacle.target_back_until_return_turn",
    "stage4.blue_obstacle.channel",
    "stage4.channel.obstacle_flow",
    "stage4.channels.three_channel_flow",
    "stage4.channels.after_channels",
    "stage4.yellow_line.fisheye_yaw",
    "stage4.yellow_line.left_fisheye_yaw",
    "stage4.yellow_line.right_fisheye_roi_shift",
    "stage4.yellow_line.rgb_yaw",
    "stage4.yellow_line.rgb_forward",
    "stage4.yellow_line.rgb_stationary_center",
    "stage4.motion.relative_with_fisheye_overlay",
    "stage4.motion.relative_pose_feedback",
    "stage4.motion.relative_move",
    "stage4.motion.relative_yaw",
    "stage4.motion.obstacle_return_height_compensation",
    "stage4.motion.limit_return_height_compensation",
    "stage4.target.scan",
    "stage4.target.resolve_attempt",
    "stage4.target.lateral_align",
    "stage4.target.forward_lateral_adjust",
    "stage4.target.until_yellow_marker",
    "stage4.target.until_bottom_yellow_marker",
    "stage4.target.until_red_ball_marker",
    "stage4.target.until_top_red_marker",
    "stage4.target.backward_until_top_red_marker",
    "stage4.target.football_safe_lateral_search",
    "stage4.orange_ball_bow",
    "stage4.speech.announce_key",
    "stage4.debug.after_channels_final_tail",
    "stage4.debug.final_rgb_lane_forward",
    "stage4.debug.limit_height_lane_head_center",
    "stage4.debug.limit_height_pass_return",
};

}  // namespace

const std::vector<BusinessPhaseDefinition>& business_phases(StageId stage) {
    switch (stage) {
        case StageId::Stage1: return kStage1Phases;
        case StageId::Stage2: return kStage2Phases;
        case StageId::Stage3: return kStage3Phases;
        case StageId::Stage4: return kStage4Phases;
        case StageId::Stage5: return kStage5Phases;
        case StageId::Stage6: return kStage6Phases;
    }
    return kStage1Phases;
}

const std::vector<std::string_view>& stage4_business_nodes() {
    return kStage4BusinessNodes;
}

std::optional<StageId> parse_stage_id(std::string_view value) {
    std::string normalized{value};
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized.starts_with("stage_")) normalized.erase(5, 1);
    if (normalized.size() == 2 && normalized[0] == 's' && normalized[1] >= '1' && normalized[1] <= '6')
        normalized = "stage" + normalized.substr(1);
    if (normalized.size() == 1 && normalized[0] >= '1' && normalized[0] <= '6')
        normalized = "stage" + normalized;
    const auto found = std::find(kStageNames.begin(), kStageNames.end(), normalized);
    if (found == kStageNames.end()) return std::nullopt;
    return static_cast<StageId>(std::distance(kStageNames.begin(), found));
}

std::string_view stage_name(StageId stage) {
    return kStageNames.at(static_cast<std::size_t>(stage));
}

std::optional<int> parse_business_phase(StageId stage, std::string_view selector) {
    if (selector.empty()) return 1;
    int numeric{};
    const auto parsed = std::from_chars(selector.data(), selector.data() + selector.size(), numeric);
    const auto& phases = business_phases(stage);
    if (parsed.ec == std::errc{} && parsed.ptr == selector.data() + selector.size())
        return numeric >= 1 && numeric <= static_cast<int>(phases.size()) ? std::optional<int>{numeric}
                                                                          : std::nullopt;
    std::string normalized{selector};
    std::replace(normalized.begin(), normalized.end(), '-', '_');
    for (std::size_t index = 0; index < phases.size(); ++index)
        if (phases[index].key == normalized) return static_cast<int>(index + 1);
    return std::nullopt;
}

BusinessEngine::BusinessEngine(BusinessEngineConfig config) : config_(std::move(config)) {
    for (std::size_t index = 0; index < states_.size(); ++index) {
        const auto stage = static_cast<StageId>(index);
        const int count = static_cast<int>(business_phases(stage).size());
        config_.start_phase[index] = std::clamp(config_.start_phase[index], 1, count);
        states_[index].phase = config_.start_phase[index] - 1;
    }
}

protocol::RobotControlCommand BusinessEngine::motion(int mode, int gait, double vx, double vy,
                                                      double wz, double body_height,
                                                      int duration_ms) {
    protocol::RobotControlCommand command;
    command.mode = static_cast<std::int8_t>(mode);
    command.gait_id = static_cast<std::int8_t>(gait);
    command.contact = 15;
    command.vel_des = {static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(wz)};
    command.pos_des[2] = static_cast<float>(body_height);
    command.step_height = {0.06F, 0.06F};
    command.duration = duration_ms;
    return command;
}

void BusinessEngine::reset(StageId stage) {
    const auto index = static_cast<std::size_t>(stage);
    states_[index] = {};
    states_[index].phase = config_.start_phase[index] - 1;
    if (stage == StageId::Stage5) depth_follower_ = {};
}

void BusinessEngine::enter_phase(StageId stage, StageState& state, int phase,
                                 std::chrono::steady_clock::time_point now) {
    const int count = static_cast<int>(business_phases(stage).size());
    state.phase = std::clamp(phase, 0, count - 1);
    state.step = 0;
    state.stable_frames = 0;
    state.lost_frames = 0;
    state.phase_ticks = 0;
    state.pass_count = 0;
    state.miss_count = 0;
    state.pose_latched = false;
    state.turn_target_latched = false;
    state.phase_started = now;
}

void BusinessEngine::next_phase(StageId stage, StageState& state,
                                std::chrono::steady_clock::time_point now) {
    enter_phase(stage, state, state.phase + 1, now);
}

void BusinessEngine::enter_step(StageState& state, int step) {
    state.step = step;
    state.stable_frames = 0;
    state.lost_frames = 0;
    state.phase_ticks = 0;
    state.pass_count = 0;
    state.miss_count = 0;
    state.pose_latched = false;
    state.turn_target_latched = false;
}

double BusinessEngine::phase_seconds(const StageState& state, const SensorFrame& frame) const {
    (void)frame;
    const double ticks = static_cast<double>(state.phase_ticks) *
                         static_cast<double>(config_.nominal_tick.count()) / 1000.0;
    return ticks;
}

void BusinessEngine::latch_segment_pose(StageState& state, const PoseSample& pose) {
    state.pose_latched = true;
    state.segment_origin_x = pose.x;
    state.segment_origin_y = pose.y;
    state.segment_origin_yaw_deg = pose.yaw_deg;
}

double BusinessEngine::segment_progress(const StageState& state, const PoseSample& pose) const {
    const double yaw = state.segment_origin_yaw_deg * CV_PI / 180.0;
    return (pose.x - state.segment_origin_x) * std::cos(yaw) +
           (pose.y - state.segment_origin_y) * std::sin(yaw);
}

BusinessDecision BusinessEngine::decision(StageId stage, const StageState& state,
                                          protocol::RobotControlCommand command,
                                          std::string action, std::string reason,
                                          bool fail_closed) const {
    const auto& phases = business_phases(stage);
    BusinessDecision result;
    result.stage = stage;
    result.phase = state.phase + 1;
    result.step = state.step;
    result.command = command;
    result.fail_closed = fail_closed;
    result.phase_key = std::string{phases.at(static_cast<std::size_t>(state.phase)).key};
    result.action = std::move(action);
    result.reason = std::move(reason);
    return result;
}

BusinessDecision BusinessEngine::hold(StageId stage, const StageState& state,
                                      std::string reason) const {
    return decision(stage, state, motion(12, 0), "stand", std::move(reason), true);
}

BusinessDecision BusinessEngine::complete(StageId stage, const StageState& state,
                                          std::string reason) const {
    auto result = decision(stage, state, motion(12, 0), "stage_complete", std::move(reason));
    result.stage_complete = true;
    return result;
}

BusinessDecision BusinessEngine::tick(StageId stage, const SensorFrame& frame) {
    auto& state = states_.at(static_cast<std::size_t>(stage));
    if (!state.initialized) {
        state.initialized = true;
        state.phase_started = frame.now;
    }
    if (frame.stop_requested) return hold(stage, state, "stop_requested");
    if (!frame.control_safe) return hold(stage, state, "control_status_unsafe");
    ++state.phase_ticks;
    switch (stage) {
        case StageId::Stage1: return stage1(frame, state);
        case StageId::Stage2: return stage2(frame, state);
        case StageId::Stage3: return stage3(frame, state);
        case StageId::Stage4: return stage4(frame, state);
        case StageId::Stage5: return stage5(frame, state);
        case StageId::Stage6: return stage6(frame, state);
    }
    return hold(stage, state, "invalid_stage");
}

}  // namespace doogle::competition
