#include "instance/competition/business_engine.hpp"

#include <algorithm>
#include <cmath>

#include "service/perception/geometry.hpp"

namespace doogle::competition {
namespace {

int target_slot(perception::VisualTarget target) {
    switch (target) {
        case perception::VisualTarget::Football: return 0;
        case perception::VisualTarget::Cola: return 1;
        case perception::VisualTarget::OrangeBall: return 2;
        default: return -1;
    }
}

perception::VisualTarget select_stage4_target(const SensorFrame& frame,
                                              const std::array<bool, 4>& completed) {
    struct Candidate {
        perception::VisualTarget target;
        double score;
    };
    std::vector<Candidate> candidates;
    const auto football = frame.football && frame.football->detected
                              ? *frame.football
                              : perception::detect_football_heuristic(frame.rgb);
    if (!completed[0] && football.detected)
        candidates.push_back({perception::VisualTarget::Football, football.score});
    const auto cola = frame.observations.cola
                          ? *frame.observations.cola
                          : perception::detect_colored_target(frame.rgb, perception::VisualTarget::Cola, 0.95);
    if (!completed[1] && cola.component.detected)
        candidates.push_back({perception::VisualTarget::Cola, cola.confidence});
    const auto orange = frame.observations.orange_ball
                            ? *frame.observations.orange_ball
                            : perception::detect_colored_target(frame.rgb, perception::VisualTarget::OrangeBall, 0.95);
    if (!completed[2] && orange.component.detected)
        candidates.push_back({perception::VisualTarget::OrangeBall, orange.confidence});
    if (candidates.empty()) return perception::VisualTarget::None;
    return std::max_element(candidates.begin(), candidates.end(),
                            [](const auto& lhs, const auto& rhs) { return lhs.score < rhs.score; })
        ->target;
}

std::string_view target_action_name(perception::VisualTarget target) {
    switch (target) {
        case perception::VisualTarget::Football: return "football";
        case perception::VisualTarget::Cola: return "cola";
        case perception::VisualTarget::OrangeBall: return "orange_ball";
        default: return "none";
    }
}

}  // namespace

BusinessDecision BusinessEngine::stage4(const SensorFrame& frame, StageState& state) {
    auto pose_move = [&](double forward_m, double lateral_m, std::string action,
                         double body_height = 0.235, int mode = 11,
                         int gait = 27) -> BusinessDecision {
        if (!frame.pose) return hold(StageId::Stage4, state, "pose_not_ready");
        if (!state.pose_latched) latch_segment_pose(state, *frame.pose);
        const double yaw = state.segment_origin_yaw_deg * CV_PI / 180.0;
        const double dx = frame.pose->x - state.segment_origin_x;
        const double dy = frame.pose->y - state.segment_origin_y;
        const double forward = dx * std::cos(yaw) + dy * std::sin(yaw);
        const double lateral = -dx * std::sin(yaw) + dy * std::cos(yaw);
        const double forward_error = forward_m - forward;
        const double lateral_error = lateral_m - lateral;
        if (std::hypot(forward_error, lateral_error) <= 0.04) {
            enter_step(state, state.step + 1);
            return decision(StageId::Stage4, state, motion(12, 0), std::move(action),
                            "pose_relative_target_reached");
        }
        const double yaw_error = perception::normalize_angle_degrees(
            state.segment_origin_yaw_deg - frame.pose->yaw_deg);
        const double vx = std::clamp(0.75 * forward_error, -0.40, 0.40);
        const double vy = std::clamp(0.75 * lateral_error, -0.40, 0.40);
        return decision(StageId::Stage4, state,
                        motion(mode, gait, vx, vy,
                               std::clamp(0.025 * yaw_error, -0.20, 0.20),
                               body_height, 500),
                        std::move(action), "pose_relative_closed_loop");
    };

    auto turn = [&](double relative_deg, std::string action) -> BusinessDecision {
        if (!frame.pose) return hold(StageId::Stage4, state, "pose_not_ready_for_turn");
        if (!state.turn_target_latched) {
            state.target_yaw_deg = perception::normalize_angle_degrees(frame.pose->yaw_deg + relative_deg);
            state.turn_target_latched = true;
        }
        const double error = perception::normalize_angle_degrees(state.target_yaw_deg - frame.pose->yaw_deg);
        state.stable_frames = std::abs(error) <= 3.0 ? state.stable_frames + 1 : 0;
        if (state.stable_frames >= 3) enter_step(state, state.step + 1);
        const double magnitude = std::clamp(std::abs(error) * 0.025, 0.06, 0.90);
        return decision(StageId::Stage4, state,
                        motion(11, 27, 0.0, 0.0, std::copysign(magnitude, error)),
                        std::move(action), "pose_relative_yaw_closed_loop");
    };

    auto fisheye_align = [&](bool left_side, std::string action) -> BusinessDecision {
        const auto line = left_side
                              ? (frame.observations.left_yellow_line
                                     ? *frame.observations.left_yellow_line
                                     : perception::analyze_yellow_line(frame.left_fisheye, 0.55))
                              : (frame.observations.right_yellow_line
                                     ? *frame.observations.right_yellow_line
                                     : perception::analyze_yellow_line(frame.right_fisheye, 0.55));
        if (!line.detected) {
            ++state.lost_frames;
            if (state.lost_frames >= 20 && config_.continue_after_soft_timeout) {
                enter_step(state, state.step + 1);
                auto result = decision(StageId::Stage4, state, motion(12, 0), std::move(action),
                                       "fisheye_alignment_soft_timeout");
                result.degraded = true;
                return result;
            }
            return decision(StageId::Stage4, state, motion(11, 27, 0.0, 0.0, 0.06),
                            std::move(action), "yellow_tangent_search");
        }
        state.lost_frames = 0;
        state.stable_frames = std::abs(line.angle_deg) <= 4.0 ? state.stable_frames + 1 : 0;
        if (state.stable_frames >= 8) enter_step(state, state.step + 1);
        return decision(StageId::Stage4, state,
                        motion(11, 27, 0.0, 0.0,
                               std::clamp(-0.16 * line.angle_deg, -0.22, 0.22)),
                        std::move(action), "yellow_tangent_yaw_correction");
    };

    // Selectable resume phases 14-16 all enter the same channel flow at p0.
    if (state.phase >= 13) {
        state.channel = state.phase - 13;
        enter_phase(StageId::Stage4, state, 8, frame.now);
        return decision(StageId::Stage4, state, motion(12, 0), "resume_channel_p0",
                        "selected_business_phase_resume");
    }

    // Configuration and optional diagnostic phases are explicit and observable,
    // but they do not duplicate infrastructure owned outside the business engine.
    if (state.phase <= 2 || (state.phase >= 4 && state.phase <= 6)) {
        const std::string action = std::string{business_phases(StageId::Stage4)[state.phase].key};
        next_phase(StageId::Stage4, state, frame.now);
        return decision(StageId::Stage4, state, motion(12, 0), action,
                        "business_contract_ready");
    }

    if (state.phase == 3) {
        if (!frame.pose) return hold(StageId::Stage4, state, "pose_not_ready");
        next_phase(StageId::Stage4, state, frame.now);
        return decision(StageId::Stage4, state, motion(12, 0), "lcm_prepare_pose",
                        "stand_and_pose_ready");
    }

    if (state.phase == 7) {
        auto result = pose_move(0.30, 0.0, "entry_move");
        if (state.step >= 1) {
            state.channel = 0;
            next_phase(StageId::Stage4, state, frame.now);
        }
        return result;
    }

    auto start_target_action = [&](int continuation_step) {
        state.continuation_step = continuation_step;
        switch (state.current_target) {
            case perception::VisualTarget::Football: enter_step(state, 100); break;
            case perception::VisualTarget::Cola: enter_step(state, 110); break;
            case perception::VisualTarget::OrangeBall: enter_step(state, 120); break;
            default: enter_step(state, continuation_step); break;
        }
    };

    auto finish_target_action = [&]() {
        const int slot = target_slot(state.current_target);
        if (slot >= 0) state.completed_target_classes[static_cast<std::size_t>(slot)] = true;
        state.target_action_sent = state.current_target != perception::VisualTarget::None;
        state.current_target = perception::VisualTarget::None;
        enter_step(state, state.continuation_step);
    };

    // Exact target action sequences shared by the limit-bar and blue-obstacle branches.
    if (state.phase == 8 && state.step >= 100) {
        if (state.step == 100) return pose_move(0.0, 0.0, "football_forward");
        if (state.step == 101) return turn(90.0, "football_left_turn");
        if (state.step == 102) return pose_move(0.0, -0.20, "football_right_lateral");
        if (state.step == 103) {
            if (phase_seconds(state, frame) >= 0.5) enter_step(state, 104);
            return decision(StageId::Stage4, state, motion(12, 0), "football_stand", "target_action_settle");
        }
        if (state.step == 104) return pose_move(0.0, 0.20, "football_left_lateral");
        if (state.step == 105) return turn(-90.0, "football_right_turn");
        if (state.step == 106) {
            auto result = pose_move(0.0, 0.0, "football_back");
            if (state.step >= 107) finish_target_action();
            return result;
        }
        if (state.step == 110) return pose_move(0.50, 0.0, "cola_forward");
        if (state.step == 111) return pose_move(0.0, 0.30, "cola_left_lateral");
        if (state.step == 112) {
            if (phase_seconds(state, frame) >= 0.5) enter_step(state, 113);
            return decision(StageId::Stage4, state, motion(12, 0), "cola_stand", "target_action_settle");
        }
        if (state.step == 113) return pose_move(0.0, -0.70, "cola_right_lateral");
        if (state.step == 114) {
            if (phase_seconds(state, frame) >= 0.5) enter_step(state, 115);
            return decision(StageId::Stage4, state, motion(12, 0), "cola_right_stand", "target_action_settle");
        }
        if (state.step == 115) return pose_move(0.0, 0.30, "cola_left_again");
        if (state.step == 116) {
            auto result = pose_move(-0.50, 0.0, "cola_back");
            if (state.step >= 117) finish_target_action();
            return result;
        }
        if (state.step == 120) return pose_move(0.30, 0.0, "orange_ball_forward");
        if (state.step == 121) {
            if (phase_seconds(state, frame) >= 0.8) enter_step(state, 122);
            return decision(StageId::Stage4, state, motion(16, 6, 0.0, 0.0, 0.0, 0.235, 800),
                            "orange_ball_jump", "table_jump_action");
        }
        if (state.step == 122) {
            auto result = pose_move(-0.30, 0.0, "orange_ball_back");
            if (state.step >= 123) finish_target_action();
            return result;
        }
    }

    if (state.phase == 8) {
        auto advance_channel = [&]() {
            ++state.channel;
            state.attempt = 0;
            if (state.channel >= 3) {
                next_phase(StageId::Stage4, state, frame.now);
            } else {
                enter_step(state, 0);
            }
        };

        if (state.step == 0) {
            if (frame.rgb.empty()) return hold(StageId::Stage4, state, "rgb_not_ready");
            const auto limit = frame.observations.limit_bar
                                   ? *frame.observations.limit_bar
                                   : perception::detect_limit_bar(frame.rgb);
            const auto blue = frame.observations.blue_cube
                                  ? *frame.observations.blue_cube
                                  : perception::detect_colored_target(
                                        frame.rgb, perception::VisualTarget::BlueCube, 0.95);
            if (limit.detected) {
                enter_step(state, 1);
                auto result = decision(StageId::Stage4, state, motion(12, 0), "limit_height_selected",
                                       "entry_obstacle_scan");
                result.speech = "limit_height";
                return result;
            }
            if (blue.component.detected) {
                enter_step(state, 20);
                auto result = decision(StageId::Stage4, state, motion(12, 0), "blue_cube_selected",
                                       "entry_obstacle_scan");
                result.speech = "blue_cube";
                return result;
            }
            ++state.lost_frames;
            if (state.lost_frames >= 60 && config_.continue_after_soft_timeout) {
                enter_step(state, state.channel == 0 ? 1 : 20);
                auto result = decision(StageId::Stage4, state, motion(12, 0), "obstacle_fallback",
                                       "entry_scan_soft_timeout");
                result.degraded = true;
                return result;
            }
            return decision(StageId::Stage4, state,
                            motion(11, 27, 0.02, state.phase_ticks % 80 < 40 ? 0.04 : -0.04),
                            "entry_obstacle_scan", "seek_limit_height_or_blue_cube");
        }

        // Limit-bar channel: center -> top-red trigger -> pre-align -> low crawl ->
        // post-align -> target scan -> p2 -> target action -> back -> crawl return.
        if (state.step == 1) {
            const auto limit = frame.observations.limit_bar
                                   ? *frame.observations.limit_bar
                                   : perception::detect_limit_bar(frame.rgb);
            if (!limit.detected) return hold(StageId::Stage4, state, "limit_height_lost_before_center");
            const double center = (limit.beam.x + limit.beam.width * 0.5) /
                                  std::max(1, frame.rgb.cols);
            const double error = center - 0.5;
            state.stable_frames = std::abs(error) <= 0.04 ? state.stable_frames + 1 : 0;
            if (state.stable_frames >= 5) enter_step(state, 2);
            return decision(StageId::Stage4, state,
                            motion(11, 27, limit.near ? 0.03 : 0.10,
                                   std::clamp(-0.30 * error, -0.08, 0.08)),
                            "limit_height_lane_head_center", "red_beam_and_posts_centering");
        }
        if (state.step == 2) {
            const auto red = frame.observations.top_red
                                 ? *frame.observations.top_red
                                 : perception::detect_top_red_marker(frame.rgb, 0.20, 0.50);
            state.stable_frames = red.detected ? state.stable_frames + 1 : 0;
            if (state.stable_frames >= 3) enter_step(state, 3);
            return decision(StageId::Stage4, state, motion(11, 27, 0.08),
                            "limit_height_until_top_red", red.detected ? "top_red_trigger_stable"
                                                                        : "approach_top_red_trigger");
        }
        if (state.step == 3) return fisheye_align(true, "limit_height_pre_crawl_fisheye_align");
        if (state.step == 4) {
            auto result = pose_move(0.70, 0.0, "limit_height_low_crawl", -0.12, 62, 83);
            result.command.rpy_des[1] = 0.22F;
            result.command.step_height = {0.035F, 0.035F};
            return result;
        }
        if (state.step == 5) {
            if (phase_seconds(state, frame) >= 0.5) enter_step(state, 6);
            return decision(StageId::Stage4, state, motion(12, 0), "limit_height_post_pass_stand",
                            "recover_normal_height");
        }
        if (state.step == 6) return fisheye_align(true, "limit_height_post_pass_fisheye_align");
        if (state.step == 7) {
            const auto target = select_stage4_target(frame, state.completed_target_classes);
            if (target != perception::VisualTarget::None) {
                state.current_target = target;
                state.target_is_fallback = false;
                enter_step(state, 8);
            } else if (++state.attempt >= 60 && config_.continue_after_soft_timeout) {
                for (int slot = 0; slot < 3; ++slot) {
                    if (!state.completed_target_classes[static_cast<std::size_t>(slot)]) {
                        state.current_target = slot == 0 ? perception::VisualTarget::Football
                                               : slot == 1 ? perception::VisualTarget::Cola
                                                           : perception::VisualTarget::OrangeBall;
                        break;
                    }
                }
                state.target_is_fallback = true;
                enter_step(state, 8);
            }
            auto result = decision(StageId::Stage4, state, motion(12, 0), "limit_height_target_scan",
                                   target == perception::VisualTarget::None ? "scan_remaining_targets"
                                                                           : "target_recognized");
            result.target = state.current_target;
            if (target != perception::VisualTarget::None)
                result.speech = std::string{target_action_name(target)};
            return result;
        }
        if (state.step == 8) return pose_move(3.40, 0.0, "limit_height_p0_to_p2", 0.20);
        if (state.step == 9) {
            start_target_action(10);
            auto result = decision(StageId::Stage4, state, motion(12, 0), "p2_target_action_start",
                                   std::string{target_action_name(state.current_target)});
            result.target = state.current_target;
            return result;
        }
        if (state.step == 10) return fisheye_align(true, "limit_height_before_p2_return_align");
        if (state.step == 11) return pose_move(-2.70, 0.0, "limit_height_p2_to_p1_back");
        if (state.step == 12) return fisheye_align(true, "limit_height_before_low_crawl_back_align");
        if (state.step == 13) {
            auto result = pose_move(-0.70, 0.0, "limit_height_low_crawl_back", -0.12, 62, 84);
            result.command.rpy_des[1] = 0.22F;
            result.command.step_height = {0.035F, 0.035F};
            return result;
        }
        if (state.step == 14) {
            advance_channel();
            return decision(StageId::Stage4, state, motion(12, 0), "limit_height_channel_complete",
                            "pass_target_return_complete");
        }

        // Blue obstacle: forced outside channels, diagonal bypass, target p2,
        // exact return path, height compensation and channel transition.
        if (state.step == 20) {
            const auto blue = frame.observations.blue_cube
                                  ? *frame.observations.blue_cube
                                  : perception::detect_colored_target(
                                        frame.rgb, perception::VisualTarget::BlueCube, 0.95);
            if (!blue.component.detected) return hold(StageId::Stage4, state, "blue_cube_lost_before_center");
            const double error = blue.component.center_x_ratio - 0.5;
            state.stable_frames = std::abs(error) < 0.05 ? state.stable_frames + 1 : 0;
            if (state.stable_frames >= 3) enter_step(state, 21);
            return decision(StageId::Stage4, state,
                            motion(11, 27, blue.component.area_ratio < 0.05 ? 0.08 : 0.02,
                                   std::clamp(-0.30 * error, -0.08, 0.08)),
                            "blue_cube_entry_center", "blue_obstacle_visual_servo");
        }
        if (state.step == 21) {
            if (state.channel == 0) state.last_y = 0.80;
            else if (state.channel == 2) state.last_y = -0.80;
            else if (frame.observations.dotted_line_right && *frame.observations.dotted_line_right)
                state.last_y = -0.80;
            else
                state.last_y = 0.80;
            enter_step(state, 22);
        }
        if (state.step == 22) return pose_move(0.0, state.last_y, "blue_cube_bypass_lateral");
        if (state.step == 23) return pose_move(1.45, 0.0, "blue_cube_bypass_forward");
        if (state.step == 24) return turn(state.last_y > 0.0 ? -60.0 : 60.0, "blue_cube_diagonal_turn");
        if (state.step == 25) return pose_move(0.95, 0.0, "blue_cube_diagonal_forward");
        if (state.step == 26) return turn(state.last_y > 0.0 ? 60.0 : -60.0, "blue_cube_restore_yaw");
        if (state.step == 27) {
            const auto target = select_stage4_target(frame, state.completed_target_classes);
            if (target != perception::VisualTarget::None) {
                state.current_target = target;
                state.target_is_fallback = false;
                enter_step(state, 28);
            } else if (++state.attempt >= 60 && config_.continue_after_soft_timeout) {
                for (int slot = 0; slot < 3; ++slot)
                    if (!state.completed_target_classes[static_cast<std::size_t>(slot)]) {
                        state.current_target = slot == 0 ? perception::VisualTarget::Football
                                               : slot == 1 ? perception::VisualTarget::Cola
                                                           : perception::VisualTarget::OrangeBall;
                        break;
                    }
                state.target_is_fallback = true;
                enter_step(state, 28);
            }
            auto result = decision(StageId::Stage4, state, motion(12, 0), "blue_cube_target_scan",
                                   target == perception::VisualTarget::None ? "scan_remaining_targets"
                                                                           : "target_recognized");
            result.target = state.current_target;
            if (target != perception::VisualTarget::None)
                result.speech = std::string{target_action_name(target)};
            return result;
        }
        if (state.step == 28) return fisheye_align(true, "blue_target_before_p1_to_p2_align");
        if (state.step == 29) return pose_move(3.40, 0.0, "blue_target_p0_to_p2");
        if (state.step == 30) {
            start_target_action(31);
            auto result = decision(StageId::Stage4, state, motion(12, 0), "p2_target_action_start",
                                   std::string{target_action_name(state.current_target)});
            result.target = state.current_target;
            return result;
        }
        if (state.step == 31) return fisheye_align(true, "blue_target_before_return_align");
        if (state.step == 32) return pose_move(-2.00, 0.0, "blue_target_back_to_p1");
        if (state.step == 33) return turn(state.last_y > 0.0 ? -60.0 : 60.0, "blue_return_diagonal_turn");
        if (state.step == 34) return pose_move(-0.95, 0.0, "blue_return_diagonal_back");
        if (state.step == 35) return turn(state.last_y > 0.0 ? 60.0 : -60.0, "blue_return_restore_yaw");
        if (state.step == 36) return fisheye_align(state.last_y > 0.0, "blue_return_fisheye_align");
        if (state.step == 37) return pose_move(-1.55, -state.last_y, "blue_return_straight_and_lateral");
        if (state.step == 38) {
            advance_channel();
            return decision(StageId::Stage4, state, motion(12, 0), "blue_cube_channel_complete",
                            "bypass_target_return_complete");
        }
        return hold(StageId::Stage4, state, "invalid_obstacle_substep");
    }

    if (state.phase == 9) {
        const auto target = select_stage4_target(frame, state.completed_target_classes);
        if (target == perception::VisualTarget::None || phase_seconds(state, frame) >= 1.0) {
            next_phase(StageId::Stage4, state, frame.now);
            return decision(StageId::Stage4, state, motion(12, 0), "entry_vision_scan_complete",
                            "remaining_target_scan_settled");
        }
        auto result = decision(StageId::Stage4, state, motion(12, 0), "entry_vision_scan",
                               "remaining_target_visible");
        result.target = target;
        return result;
    }

    if (state.phase == 10) {
        const auto marker = frame.observations.right_bottom_yellow
                                ? *frame.observations.right_bottom_yellow
                                : perception::detect_bottom_yellow_marker(frame.right_fisheye, 0.18, 0.45);
        if (state.step == 0) {
            state.stable_frames = marker.detected ? state.stable_frames + 1 : 0;
            if (state.stable_frames >= 3) enter_step(state, 1);
            return decision(StageId::Stage4, state, motion(11, 27, 0.0, marker.detected ? 0.0 : 0.05),
                            "after_channels_right_roi_shift", "seek_right_yellow_roi");
        }
        if (state.step == 1) return pose_move(0.80, 0.0, "after_channels_stone_road", 0.235, 62, 81);
        if (state.step == 2) return fisheye_align(true, "after_channels_left_fisheye_align");
        auto result = pose_move(0.50, 0.0, "after_channels_final_forward");
        if (state.step >= 4) next_phase(StageId::Stage4, state, frame.now);
        return result;
    }

    if (state.phase == 11) {
        const auto lane = frame.observations.rgb_lane
                              ? *frame.observations.rgb_lane
                              : perception::detect_three_line_lane(frame.rgb);
        if (!lane.detected) return hold(StageId::Stage4, state, "three_yellow_lane_not_ready");
        if (std::abs(lane.center_error) < 0.05 && std::abs(lane.heading_error) < 0.08)
            ++state.stable_frames;
        else
            state.stable_frames = 0;
        if (state.stable_frames >= 5 || phase_seconds(state, frame) >= 3.0)
            return complete(StageId::Stage4, state, "three_channels_and_final_lane_complete");
        return decision(StageId::Stage4, state,
                        motion(11, 27, 0.12,
                               std::clamp(-0.10 * lane.center_error, -0.08, 0.08),
                               std::clamp(-0.35 * lane.heading_error, -0.22, 0.22)),
                        "final_rgb_lane_forward", "right_two_of_three_lane_control");
    }

    // The selectable limit-height pass-return test reuses the complete channel
    // implementation and terminates after one channel.
    if (state.phase == 12) {
        state.channel = 0;
        enter_phase(StageId::Stage4, state, 8, frame.now);
        return decision(StageId::Stage4, state, motion(12, 0), "limit_height_pass_return_test",
                        "reuse_production_limit_height_flow");
    }

    return hold(StageId::Stage4, state, "invalid_business_phase");
}

}  // namespace doogle::competition
