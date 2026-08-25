#include "instance/competition/business_engine.hpp"

#include <algorithm>
#include <cmath>

#include "service/perception/geometry.hpp"

namespace doogle::competition {
namespace {

struct StereoBall {
    perception::ColorDetection left;
    perception::ColorDetection right;
    perception::VisualTarget target{perception::VisualTarget::None};
    bool any{};
    bool centered{};
    bool strict_centered{};
    bool area_close{};
};

StereoBall observe_stereo_ball(const SensorFrame& frame) {
    StereoBall result;
    const auto left_blue = perception::detect_colored_target(
        frame.left_fisheye, perception::VisualTarget::BlueBall, 0.667);
    const auto right_blue = perception::detect_colored_target(
        frame.right_fisheye, perception::VisualTarget::BlueBall, 0.667);
    const auto left_red = perception::detect_colored_target(
        frame.left_fisheye, perception::VisualTarget::RedBall, 0.667);
    const auto right_red = perception::detect_colored_target(
        frame.right_fisheye, perception::VisualTarget::RedBall, 0.667);
    const double blue_area = left_blue.component.area_ratio + right_blue.component.area_ratio;
    const double red_area = left_red.component.area_ratio + right_red.component.area_ratio;
    if (blue_area >= red_area) {
        result.left = left_blue;
        result.right = right_blue;
        result.target = perception::VisualTarget::BlueBall;
    } else {
        result.left = left_red;
        result.right = right_red;
        result.target = perception::VisualTarget::RedBall;
    }
    result.any = result.left.component.detected || result.right.component.detected;
    const auto centered = [](const perception::ColorDetection& detection, double tolerance) {
        return detection.component.detected &&
               std::abs(detection.component.center_x_ratio - 0.5) <= tolerance;
    };
    result.centered = centered(result.left, 0.05) || centered(result.right, 0.05);
    result.strict_centered = centered(result.left, 0.03) && centered(result.right, 0.03);
    const double max_area = std::max(result.left.component.area_ratio, result.right.component.area_ratio);
    result.area_close = max_area <= 1e-9 ||
                        std::abs(result.left.component.area_ratio - result.right.component.area_ratio) /
                                max_area <=
                            0.10;
    return result;
}

double yaw_error(double target, double current) {
    return perception::normalize_angle_degrees(target - current);
}

}  // namespace

BusinessDecision BusinessEngine::stage2(const SensorFrame& frame, StageState& state) {
    auto finish_phase = [&]() {
        state.cycle = 0;
        state.current_target = perception::VisualTarget::None;
        next_phase(StageId::Stage2, state, frame.now);
    };

    auto pose_move = [&](double distance, double vx, std::string action,
                         std::string reason) -> std::optional<BusinessDecision> {
        if (!frame.pose) return hold(StageId::Stage2, state, "pose_not_ready");
        if (!state.pose_latched) latch_segment_pose(state, *frame.pose);
        const double progress = segment_progress(state, *frame.pose);
        const bool reached = distance >= 0.0 ? progress >= distance - 0.03 : progress <= distance + 0.03;
        if (reached) {
            enter_step(state, state.step + 1);
            return decision(StageId::Stage2, state, motion(12, 0), std::move(action),
                            "pose_distance_reached");
        }
        const double remaining = std::abs(distance - progress);
        const double speed = remaining < std::max(0.12, std::abs(distance) * 0.35)
                                 ? std::copysign(0.10, vx)
                                 : vx;
        const double yaw = yaw_error(state.segment_origin_yaw_deg, frame.pose->yaw_deg);
        return decision(StageId::Stage2, state,
                        motion(11, 27, speed, 0.0, std::clamp(0.025 * yaw, -0.08, 0.08)),
                        std::move(action), std::move(reason));
    };

    auto lateral_move = [&](double distance, std::string action) -> std::optional<BusinessDecision> {
        if (!frame.pose) return hold(StageId::Stage2, state, "pose_not_ready");
        if (!state.pose_latched) latch_segment_pose(state, *frame.pose);
        const double yaw = state.segment_origin_yaw_deg * CV_PI / 180.0;
        const double dx = frame.pose->x - state.segment_origin_x;
        const double dy = frame.pose->y - state.segment_origin_y;
        const double lateral = -dx * std::sin(yaw) + dy * std::cos(yaw);
        const bool reached = distance >= 0.0 ? lateral >= distance - 0.025 : lateral <= distance + 0.025;
        if (reached) {
            enter_step(state, state.step + 1);
            return decision(StageId::Stage2, state, motion(12, 0), std::move(action),
                            "pose_lateral_reached");
        }
        const double remaining = std::abs(distance - lateral);
        const double vy = std::copysign(remaining < 0.15 ? 0.10 : 0.20, distance);
        const double error = yaw_error(state.segment_origin_yaw_deg, frame.pose->yaw_deg);
        return decision(StageId::Stage2, state,
                        motion(11, 27, 0.0, vy, std::clamp(0.025 * error, -0.08, 0.08)),
                        std::move(action), "pose_lateral_closed_loop");
    };

    auto turn = [&](double relative, std::string action) -> BusinessDecision {
        if (!frame.pose) return hold(StageId::Stage2, state, "pose_not_ready_for_turn");
        if (!state.turn_target_latched) {
            state.target_yaw_deg = perception::normalize_angle_degrees(frame.pose->yaw_deg + relative);
            state.turn_target_latched = true;
        }
        const double error = yaw_error(state.target_yaw_deg, frame.pose->yaw_deg);
        state.stable_frames = std::abs(error) <= 3.0 ? state.stable_frames + 1 : 0;
        if (state.stable_frames >= 3) enter_step(state, state.step + 1);
        const double magnitude = std::clamp(std::abs(error) * 0.025, 0.08, 0.30);
        return decision(StageId::Stage2, state,
                        motion(11, 27, 0.0, 0.0, std::copysign(magnitude, error)),
                        std::move(action), "pose_relative_yaw_closed_loop");
    };

    if (state.phase == 0) {
        if (!frame.pose) return hold(StageId::Stage2, state, "pose_not_ready");
        if (phase_seconds(state, frame) < 2.2)
            return decision(StageId::Stage2, state, motion(12, 0), "startup_stand_reference",
                            "stand_and_capture_reference");
        state.origin_x = frame.pose->x;
        state.origin_y = frame.pose->y;
        state.origin_yaw_deg = frame.pose->yaw_deg;
        finish_phase();
        return decision(StageId::Stage2, state, motion(12, 0), "startup_reference_captured",
                        "pose_reference_ready");
    }

    if (state.phase == 1) {
        if (frame.left_fisheye.empty() || frame.right_fisheye.empty())
            return hold(StageId::Stage2, state, "stereo_fisheye_not_ready");
        const auto ball = observe_stereo_ball(frame);
        if (!ball.any) {
            state.stable_frames = 0;
            return decision(StageId::Stage2, state, motion(11, 27, 0.0, 0.0, 0.08),
                            "dual_fisheye_correction", "no_center_ball");
        }
        if (ball.strict_centered && ball.area_close) ++state.stable_frames;
        else state.stable_frames = 0;
        if (state.stable_frames >= 3) {
            finish_phase();
            return decision(StageId::Stage2, state, motion(12, 0), "dual_fisheye_centered",
                            "strict_center_and_area_close");
        }
        const auto& target = !ball.left.component.detected ? ball.right :
                             !ball.right.component.detected ? ball.left :
                             (std::abs(ball.left.component.center_x_ratio - 0.5) >=
                                      std::abs(ball.right.component.center_x_ratio - 0.5)
                                  ? ball.left
                                  : ball.right);
        const double error = target.component.center_x_ratio - 0.5;
        const double area_delta = ball.left.component.area_ratio - ball.right.component.area_ratio;
        return decision(StageId::Stage2, state,
                        motion(11, 27, 0.06, std::clamp(-area_delta * 2.0, -0.06, 0.06),
                               std::copysign(0.08, -error)),
                        "dual_fisheye_correction", "correct_to_strict_center_and_area");
    }

    if (state.phase == 2) {
        auto result = lateral_move(0.50, "start_left_shift");
        if (state.step >= 1) finish_phase();
        return *result;
    }

    auto recognition_cycle = [&](bool forward, bool shift_on_red) -> BusinessDecision {
        const double travel_vx = forward ? 0.20 : -0.20;
        if (state.step == 0) {
            if (frame.left_fisheye.empty() || frame.right_fisheye.empty())
                return hold(StageId::Stage2, state, "stereo_fisheye_not_ready");
            const auto ball = observe_stereo_ball(frame);
            if (ball.centered) ++state.stable_frames;
            else state.stable_frames = 0;
            if (ball.any) state.current_target = ball.target;
            if (state.stable_frames >= 2) enter_step(state, 1);
            auto result = decision(StageId::Stage2, state, motion(11, 27, travel_vx),
                                   "center_ball_recognition",
                                   ball.centered ? "centered_ball_stable" : "seek_centered_ball");
            result.target = state.current_target;
            return result;
        }
        if (state.step == 1) {
            const auto ball = observe_stereo_ball(frame);
            if (ball.strict_centered && ball.area_close) ++state.stable_frames;
            else state.stable_frames = 0;
            if (state.stable_frames >= 2) enter_step(state, 2);
            const double left_error = ball.left.component.detected
                                          ? ball.left.component.center_x_ratio - 0.5
                                          : 0.0;
            auto result = decision(StageId::Stage2, state,
                                   motion(11, 27, 0.06, 0.0,
                                          std::clamp(-left_error, -0.08, 0.08)),
                                   "cycle_dual_fisheye_correction",
                                   ball.any ? "strict_center_and_area" : "ball_temporarily_lost");
            result.target = state.current_target;
            return result;
        }
        if (state.step == 2) {
            if (shift_on_red && state.current_target == perception::VisualTarget::RedBall) {
                const auto ball = observe_stereo_ball(frame);
                const bool red_left = ball.left.component.detected &&
                                      (!ball.right.component.detected ||
                                       ball.left.component.center_x_ratio < ball.right.component.center_x_ratio);
                state.last_y = red_left ? 0.55 : -0.55;
                auto result = lateral_move(state.last_y, "red_ball_side_shift");
                return *result;
            }
            enter_step(state, 4);
        }
        if (state.step == 3) {
            auto result = lateral_move(-state.last_y, "red_ball_return_shift");
            return *result;
        }
        if (state.step == 4) {
            auto result = pose_move(forward ? 0.35 : -0.35, forward ? 0.20 : -0.20,
                                    "clear_current_ball", "post_recognition_clearance");
            if (state.step >= 5) {
                ++state.cycle;
                state.current_target = perception::VisualTarget::None;
                enter_step(state, 0);
            }
            return *result;
        }
        enter_step(state, 0);
        return decision(StageId::Stage2, state, motion(12, 0), "cycle_reset", "internal_cycle_reset");
    };

    if (state.phase == 3 || state.phase == 5 || state.phase == 6) {
        const bool forward = state.phase != 5;
        const bool shift_red = state.phase != 6;
        if (state.phase == 6 && state.cycle == 0 && state.step == 0 && !state.pose_latched && state.attempt == 0) {
            state.attempt = 1;
            auto result = pose_move(-0.25, -0.20, "post_backward_entry_back",
                                    "establish_new_forward_reference");
            if (state.step >= 1) enter_step(state, 0);
            return *result;
        }
        if (state.cycle >= 4) {
            finish_phase();
            state.attempt = 0;
            return decision(StageId::Stage2, state, motion(12, 0), "recognition_cycles_complete",
                            "four_center_ball_cycles_complete");
        }
        return recognition_cycle(forward, shift_red);
    }

    if (state.phase == 4) {
        if (state.step == 0) return *pose_move(0.25, 0.20, "final_exit_forward", "clear_ball_grid");
        if (state.step == 1) return turn(90.0, "final_exit_left_turn");
        if (state.step == 2) {
            if (!frame.pose) return hold(StageId::Stage2, state, "pose_not_ready");
            if (!state.pose_latched) latch_segment_pose(state, *frame.pose);
            const double progress = segment_progress(state, *frame.pose);
            const auto line = frame.observations.right_yellow_line
                                  ? *frame.observations.right_yellow_line
                                  : perception::analyze_yellow_line(frame.right_fisheye, 0.66);
            const double vy = line.detected
                                  ? std::clamp(-(line.bottom_x - frame.right_fisheye.cols * 0.5) /
                                                   std::max(1, frame.right_fisheye.cols) * 0.08,
                                               -0.04, 0.04)
                                  : 0.0;
            if (progress >= 2.22) enter_step(state, 3);
            return decision(StageId::Stage2, state, motion(11, 27, progress > 2.0 ? 0.10 : 0.20, vy),
                            "final_exit_left_forward", line.detected ? "right_yellow_lateral_correction"
                                                                     : "pose_forward_closed_loop");
        }
        auto result = turn(-90.0, "final_exit_right_turn");
        if (state.step >= 4) finish_phase();
        return result;
    }

    if (state.phase == 7) {
        if (state.step == 0) return *pose_move(0.37, 0.20, "left_yellow_roi_entry", "pose_forward_closed_loop");
        if (state.step == 1) {
            const auto marker = frame.observations.left_bottom_yellow
                                    ? *frame.observations.left_bottom_yellow
                                    : perception::detect_bottom_yellow_marker(frame.left_fisheye, 0.05, 0.02);
            state.stable_frames = marker.detected ? state.stable_frames + 1 : 0;
            if (state.stable_frames >= 3) enter_step(state, 2);
            return decision(StageId::Stage2, state,
                            motion(11, 27, 0.0, marker.detected ? 0.0 : 0.20),
                            "left_yellow_roi_lateral_shift",
                            marker.detected ? "left_yellow_roi_stable" : "seek_left_yellow_roi");
        }
        auto result = pose_move(0.50, 0.20, "left_yellow_roi_exit_forward", "leave_stage2");
        if (state.step >= 3) return complete(StageId::Stage2, state, "left_yellow_roi_exit_complete");
        return *result;
    }

    return hold(StageId::Stage2, state, "invalid_business_phase");
}

}  // namespace doogle::competition
