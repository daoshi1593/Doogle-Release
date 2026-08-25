#include "instance/competition/business_engine.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "service/perception/geometry.hpp"

namespace doogle::competition {

BusinessDecision BusinessEngine::stage6(const SensorFrame& frame, StageState& state) {
    auto football_observation = [&]() {
        if (frame.football && frame.football->detected) return *frame.football;
        return perception::detect_football_heuristic(frame.rgb);
    };

    auto turn = [&](double relative_deg, std::string action) -> BusinessDecision {
        if (!frame.pose) return hold(StageId::Stage6, state, "pose_not_ready_for_imu_turn");
        if (!state.turn_target_latched) {
            state.target_yaw_deg = perception::normalize_angle_degrees(frame.pose->yaw_deg + relative_deg);
            state.turn_target_latched = true;
        }
        const double error = perception::normalize_angle_degrees(state.target_yaw_deg - frame.pose->yaw_deg);
        state.stable_frames = std::abs(error) <= 2.0 ? state.stable_frames + 1 : 0;
        if (state.stable_frames >= 3) enter_step(state, state.step + 1);
        const double magnitude = std::clamp(std::abs(error) * 0.018, 0.05, 0.35);
        return decision(StageId::Stage6, state,
                        motion(11, 27, 0.0, 0.0, std::copysign(magnitude, error)),
                        std::move(action), "imu_pid_relative_turn");
    };

    if (state.phase == 0) {
        if (frame.rgb.empty() || !frame.pose)
            return hold(StageId::Stage6, state, "rgb_or_pose_not_ready");
        if (!config_.stage6_allow_partial_range && frame.depth.empty() && !frame.tof)
            return hold(StageId::Stage6, state, "depth_and_tof_not_ready");
        next_phase(StageId::Stage6, state, frame.now);
        return decision(StageId::Stage6, state, motion(12, 0), "startup_sensors",
                        "rgb_pose_and_range_ready");
    }

    if (state.phase == 1) {
        if (frame.rgb.empty() || !frame.pose)
            return hold(StageId::Stage6, state, "rgb_or_pose_not_ready");
        if (state.step == 0) {
            const auto football = football_observation();
            if (!football.detected) {
                state.stable_frames = 0;
                return decision(StageId::Stage6, state, motion(11, 27, 0.0, -0.06),
                                "search_align_ball", "lateral_search_no_ball");
            }
            const double center = football.center.x / std::max(1, frame.rgb.cols);
            if (center >= 0.25 && center <= 0.38) ++state.stable_frames;
            else state.stable_frames = 0;
            if (state.stable_frames >= 1) enter_step(state, 1);
            auto result = decision(StageId::Stage6, state,
                                   motion(11, 27, 0.0,
                                          center < 0.25 ? 0.04 : center > 0.38 ? -0.04 : 0.0),
                                   "search_align_ball", "rgb_center_window_control");
            result.target = perception::VisualTarget::Football;
            return result;
        }
        if (phase_seconds(state, frame) < 1.0)
            return decision(StageId::Stage6, state, motion(12, 0), "found_ball_stand",
                            "alignment_settle");
        next_phase(StageId::Stage6, state, frame.now);
        return decision(StageId::Stage6, state, motion(12, 0), "search_align_complete",
                        "football_locked");
    }

    if (state.phase == 2) {
        if (!frame.pose) return hold(StageId::Stage6, state, "pose_not_ready");
        if (state.step == 0) {
            if (!state.pose_latched) latch_segment_pose(state, *frame.pose);
            const double progress = segment_progress(state, *frame.pose);
            if (progress >= 1.57) enter_step(state, 1);
            return decision(StageId::Stage6, state, motion(11, 27, 0.12),
                            "fixed_approach_after_ball", "pose_distance_1_6m");
        }
        auto result = turn(90.0, "approach_transition_imu_turn");
        if (state.step >= 2) next_phase(StageId::Stage6, state, frame.now);
        return result;
    }

    if (state.phase == 3) {
        if (frame.rgb.empty() || !frame.pose)
            return hold(StageId::Stage6, state, "rgb_or_pose_not_ready");
        const auto football = football_observation();
        std::optional<perception::DepthBallPose> depth_pose;
        if (football.detected && !frame.depth.empty())
            depth_pose = perception::calculate_depth_ball_pose(
                frame.depth, {football.center.x, football.center.y, football.radius}, frame.rgb.size());
        std::optional<perception::TofBallPose> tof_pose;
        std::optional<perception::TofRampObservation> ramp;
        if (frame.tof) {
            tof_pose = perception::process_tof_ball(*frame.tof);
            ramp = perception::evaluate_tof_ramp(*frame.tof);
        }
        double boundary_y{}, boundary_slope{};
        const bool yellow_boundary = frame.observations.yellow_finish_boundary
                                         ? *frame.observations.yellow_finish_boundary
                                         : perception::detect_yellow_boundary(
                                               frame.rgb, &boundary_y, &boundary_slope);
        const bool range_boundary = ramp && ramp->reached;
        const bool reached = yellow_boundary &&
                             (range_boundary || (config_.stage6_allow_partial_range && !frame.tof));
        state.pass_count = reached ? state.pass_count + 1 : 0;
        if (state.pass_count >= 3) {
            next_phase(StageId::Stage6, state, frame.now);
            return decision(StageId::Stage6, state, motion(12, 0), "finish_boundary_reached",
                            "rgb_and_tof_boundary_confirmed");
        }

        if (state.step == 2) {
            if (phase_seconds(state, frame) >= 4.0) enter_step(state, 0);
            return decision(StageId::Stage6, state, motion(11, 27, -0.15),
                            "tof_lost_retreat", "reset_push_tracking");
        }

        std::optional<cv::Point2d> pose;
        if (tof_pose) pose = cv::Point2d{tof_pose->x_body.value, tof_pose->y_body.value};
        else if (depth_pose) pose = cv::Point2d{depth_pose->x_body.value, depth_pose->y_body.value};
        if (pose) {
            state.trajectory.push_back(*pose);
            if (state.trajectory.size() > 5) state.trajectory.pop_front();
        }
        cv::Point2d smooth{};
        if (!state.trajectory.empty()) {
            for (const auto& sample : state.trajectory) smooth += sample;
            smooth *= 1.0 / state.trajectory.size();
        }

        if (state.step == 1) {
            if (!tof_pose) {
                if (++state.miss_count >= 4) enter_step(state, 2);
                return decision(StageId::Stage6, state, motion(11, 27, 0.04),
                                "football_push_blind", "tof_temporarily_lost");
            }
            state.miss_count = 0;
            return decision(StageId::Stage6, state,
                            motion(11, 27, 0.04,
                                   std::clamp(-0.25 * smooth.y, -0.025, 0.025)),
                            "football_push_tof", "tof_push_control");
        }

        if (tof_pose && tof_pose->push_reached) {
            enter_step(state, 1);
            return decision(StageId::Stage6, state,
                            motion(11, 27, 0.04,
                                   std::clamp(-0.25 * tof_pose->y_body.value, -0.025, 0.025)),
                            "football_push_transition", "tof_push_ready");
        }
        if (pose) {
            const double vx = tof_pose ? std::clamp(0.20 * (smooth.x - 0.16), -0.02, 0.03)
                                       : std::clamp(0.35 * (smooth.x - 0.16), -0.12, 0.12);
            const double vy = std::clamp(-(tof_pose ? 0.25 : 0.45) * smooth.y, -0.08, 0.08);
            auto result = decision(StageId::Stage6, state, motion(11, 27, vx, vy),
                                   "football_fused_approach",
                                   tof_pose ? "tof_pose" : "rgb_depth_pose");
            result.target = perception::VisualTarget::Football;
            return result;
        }
        if (!config_.stage6_allow_partial_range)
            return hold(StageId::Stage6, state, "range_pose_temporarily_unavailable");
        return decision(StageId::Stage6, state, motion(11, 27, 0.0, -0.03),
                        "football_reacquire", "no_reliable_history");
    }

    if (state.phase == 4) {
        if (state.step == 0) {
            if (phase_seconds(state, frame) >= 3.0) enter_step(state, 1);
            return decision(StageId::Stage6, state, motion(12, 0), "finish_initial_stand",
                            "kick_sequence_prepare");
        }
        if (state.step == 1) {
            if (phase_seconds(state, frame) >= 3.75) enter_step(state, 2);
            return decision(StageId::Stage6, state, motion(11, 27, -0.08),
                            "finish_first_retreat", "timed_retreat");
        }
        if (state.step == 2) return turn(90.0, "finish_imu_turn");
        if (state.step == 3) {
            if (phase_seconds(state, frame) >= 3.0) enter_step(state, 4);
            return decision(StageId::Stage6, state, motion(11, 27, -0.15),
                            "finish_second_retreat", "timed_retreat");
        }
        if (state.step == 4) {
            if (phase_seconds(state, frame) >= 3.0) enter_step(state, 5);
            return decision(StageId::Stage6, state, motion(11, 27, 0.0, -0.10),
                            "finish_move_right", "timed_lateral_shift");
        }
        if (state.step == 5) {
            if (phase_seconds(state, frame) >= 1.0) enter_step(state, frame.tof ? 6 : 11);
            return decision(StageId::Stage6, state, motion(12, 0), "finish_final_stand",
                            "tof_kick_prepare");
        }
        if (state.step == 6) {
            if (!frame.tof) {
                enter_step(state, 11);
                return decision(StageId::Stage6, state, motion(12, 0), "tof_kick_skipped",
                                "tof_unavailable");
            }
            const auto tof = perception::process_tof_ball(*frame.tof);
            if (!tof) return decision(StageId::Stage6, state, motion(11, 27, 0.012),
                                      "tof_kick_search", "right_tof_not_active");
            if (std::abs(tof->y_body.value) <= 0.025) enter_step(state, 7);
            return decision(StageId::Stage6, state,
                            motion(11, 27, 0.0,
                                   std::clamp(-0.20 * tof->y_body.value, -0.015, 0.015)),
                            "tof_kick_lateral_align", "tof_horizontal_optimal");
        }
        if (state.step == 7 || state.step == 8) {
            if (phase_seconds(state, frame) >= 5.15) enter_step(state, state.step + 1);
            return decision(StageId::Stage6, state,
                            motion(11, 27, 0.36, 0.0, 0.0, 0.235, 5150),
                            "tof_kick_forward", state.step == 7 ? "first_kick" : "second_kick");
        }
        if (state.step == 9 || state.step == 10) {
            if (phase_seconds(state, frame) >= 0.8) enter_step(state, state.step + 1);
            return decision(StageId::Stage6, state, motion(12, 0), "after_kick_stand",
                            state.step == 9 ? "first_settle" : "second_settle");
        }
        return complete(StageId::Stage6, state, "finish_kick_sequence_complete");
    }

    return hold(StageId::Stage6, state, "invalid_business_phase");
}

}  // namespace doogle::competition
