#include "instance/competition/business_engine.hpp"

#include <algorithm>
#include <cmath>

#include "service/perception/geometry.hpp"

namespace doogle::competition {

BusinessDecision BusinessEngine::stage5(const SensorFrame& frame, StageState& state) {
    auto turn = [&](double relative_deg, std::string action) -> BusinessDecision {
        if (!frame.pose) return hold(StageId::Stage5, state, "pose_not_ready_for_turn");
        if (!state.turn_target_latched) {
            state.target_yaw_deg = perception::normalize_angle_degrees(frame.pose->yaw_deg + relative_deg);
            state.turn_target_latched = true;
        }
        const double error = perception::normalize_angle_degrees(state.target_yaw_deg - frame.pose->yaw_deg);
        state.stable_frames = std::abs(error) <= 1.5 ? state.stable_frames + 1 : 0;
        if (state.stable_frames >= 3) enter_step(state, state.step + 1);
        const double magnitude = std::clamp(std::abs(error) * 0.018, 0.05, 0.35);
        return decision(StageId::Stage5, state,
                        motion(11, 27, 0.0, 0.0, std::copysign(magnitude, error), 0.20),
                        std::move(action), "pose_relative_yaw_closed_loop");
    };

    auto lane_command = [&]() {
        if (!frame.depth.empty()) {
            const auto control = depth_follower_.update(frame.depth);
            if (control.bad_frames < 3)
                return motion(11, 27, control.vx, control.vy, control.wz, 0.20);
        }
        const auto lane = frame.observations.rgb_lane
                              ? *frame.observations.rgb_lane
                              : perception::detect_three_line_lane(frame.rgb);
        if (lane.detected)
            return motion(11, 27, 0.22,
                          std::clamp(-0.10 * lane.center_error, -0.10, 0.10),
                          std::clamp(-0.35 * lane.heading_error, -0.35, 0.35), 0.20);
        return motion(11, 27, 0.04, 0.0, 0.0, 0.20);
    };

    auto closed_loop = [&](double distance, std::string action) -> BusinessDecision {
        if (!frame.pose) return hold(StageId::Stage5, state, "pose_not_ready");
        if (frame.depth.empty() && frame.rgb.empty() && !frame.observations.rgb_lane)
            return hold(StageId::Stage5, state, "lane_sensor_not_ready");
        if (!state.pose_latched) latch_segment_pose(state, *frame.pose);
        const double progress = segment_progress(state, *frame.pose);
        const auto marker = frame.observations.right_bottom_yellow
                                ? *frame.observations.right_bottom_yellow
                                : perception::detect_bottom_yellow_marker(frame.right_fisheye, 0.12, 0.30);
        if (marker.detected) ++state.stable_frames;
        else state.stable_frames = 0;
        if (progress >= distance - 0.03 || state.stable_frames >= 3) {
            next_phase(StageId::Stage5, state, frame.now);
            return decision(StageId::Stage5, state, motion(12, 0), std::move(action),
                            state.stable_frames >= 3 ? "roi_stop" : "target_distance_reached");
        }
        auto result = decision(StageId::Stage5, state, lane_command(), std::move(action),
                               !frame.depth.empty() ? "depth_boundary_and_pose_control"
                                                    : "rgb_lane_and_pose_control");
        result.fail_closed = false;
        return result;
    };

    if (state.phase == 0) {
        if (frame.rgb.empty() && frame.depth.empty())
            return hold(StageId::Stage5, state, "camera_not_ready");
        if (phase_seconds(state, frame) < 1.0)
            return decision(StageId::Stage5, state, motion(12, 0, 0.0, 0.0, 0.0, 0.20),
                            "startup_stand_height", "stand_and_body_height_settle");
        next_phase(StageId::Stage5, state, frame.now);
        return decision(StageId::Stage5, state, motion(12, 0), "startup_sensors_ready",
                        "body_height_and_cameras_ready");
    }

    if (state.phase == 1) {
        if (state.step == 0) {
            if (!frame.pose) return hold(StageId::Stage5, state, "pose_not_ready");
            if (!state.pose_latched) latch_segment_pose(state, *frame.pose);
            const auto marker = frame.observations.right_bottom_yellow
                                    ? *frame.observations.right_bottom_yellow
                                    : perception::detect_bottom_yellow_marker(frame.right_fisheye, 0.15, 0.25);
            const double progress = segment_progress(state, *frame.pose);
            if (marker.detected) ++state.stable_frames;
            else state.stable_frames = 0;
            if (state.stable_frames >= 3 || progress >= 2.0) enter_step(state, 1);
            return decision(StageId::Stage5, state, motion(11, 27, 0.20, 0.0, 0.0, 0.20),
                            "startup_straight", marker.detected ? "startup_roi_stop_stable"
                                                                : "startup_straight_no_roi_control");
        }
        auto result = turn(90.0, "startup_roi_stop_left_turn");
        if (state.step >= 2) next_phase(StageId::Stage5, state, frame.now);
        return result;
    }

    if (state.phase == 2) return closed_loop(4.0, "first_closed_loop");

    auto tail = [&](std::string prefix) -> BusinessDecision {
        if (state.step == 0) {
            if (phase_seconds(state, frame) >= 1.0) enter_step(state, 1);
            return decision(StageId::Stage5, state, motion(11, 27, 0.0, 0.10, 0.0, 0.20),
                            prefix + "_pre_turn_left", "timed_lateral_tail");
        }
        if (state.step == 1) return turn(-74.0, prefix + "_turn");
        if (state.step == 2) {
            if (phase_seconds(state, frame) >= 1.2) enter_step(state, 3);
            return decision(StageId::Stage5, state, motion(11, 27, 0.22, 0.03, 0.0, 0.20),
                            prefix + "_post_turn_forward", "timed_forward_tail");
        }
        if (state.step == 3) {
            if (phase_seconds(state, frame) >= 0.8) enter_step(state, 4);
            return decision(StageId::Stage5, state, motion(11, 27, 0.0, 0.15, 0.0, 0.20),
                            prefix + "_post_stage_left", "timed_lateral_tail");
        }
        next_phase(StageId::Stage5, state, frame.now);
        return decision(StageId::Stage5, state, motion(12, 0), prefix + "_complete",
                        "tail_sequence_complete");
    };

    if (state.phase == 3) return tail("first_post_walk_tail");
    if (state.phase == 4) return closed_loop(3.0, "second_closed_loop");
    if (state.phase == 5) return tail("second_post_walk_tail");
    if (state.phase == 6) return closed_loop(3.0, "final_straight_walk");

    if (state.phase == 7) {
        auto result = turn(-90.0, "final_end_turn");
        if (state.step >= 1) next_phase(StageId::Stage5, state, frame.now);
        return result;
    }

    if (state.phase == 8) return closed_loop(1.25, "final_end_roi_forward");

    if (state.phase == 9) {
        if (state.step == 0) return turn(-90.0, "final_after_forward_turn");
        if (state.step == 1) {
            if (phase_seconds(state, frame) >= 1.0) enter_step(state, 2);
            return decision(StageId::Stage5, state,
                            motion(16, 1, 0.0, 0.0, 0.0, 0.25, 1000),
                            "final_forward_jump", "table_jump_action");
        }
        auto result = complete(StageId::Stage5, state, "turn_jump_and_lie_down_complete");
        result.command = motion(7, 0);
        result.action = "final_lie_down";
        return result;
    }

    return hold(StageId::Stage5, state, "invalid_business_phase");
}

}  // namespace doogle::competition
