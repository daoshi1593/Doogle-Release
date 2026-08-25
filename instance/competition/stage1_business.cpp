#include "instance/competition/business_engine.hpp"

#include <algorithm>
#include <cmath>

#include "service/perception/geometry.hpp"

namespace doogle::competition {
namespace {

bool color_frame(const cv::Mat& frame) {
    return !frame.empty() && frame.channels() == 3;
}

perception::YellowLineObservation right_line(const SensorFrame& frame) {
    if (frame.observations.right_yellow_line) return *frame.observations.right_yellow_line;
    return perception::analyze_yellow_line(frame.right_fisheye, 0.50);
}

perception::MarkerObservation right_marker(const SensorFrame& frame, double threshold) {
    if (frame.observations.right_bottom_yellow) return *frame.observations.right_bottom_yellow;
    return perception::detect_bottom_yellow_marker(frame.right_fisheye, 0.18, threshold);
}

perception::MarkerObservation left_marker(const SensorFrame& frame) {
    if (frame.observations.left_bottom_yellow) return *frame.observations.left_bottom_yellow;
    return perception::detect_bottom_yellow_marker(frame.left_fisheye, 0.34, 0.10);
}

}  // namespace

BusinessDecision BusinessEngine::stage1(const SensorFrame& frame, StageState& state) {
    constexpr double kStoneTailStart = 2.55;
    constexpr double kStoneDistance = 2.85;

    if (state.phase == 0) {
        if (!frame.pose || !color_frame(frame.right_fisheye))
            return hold(StageId::Stage1, state, "pose_or_right_fisheye_not_ready");
        next_phase(StageId::Stage1, state, frame.now);
        return decision(StageId::Stage1, state, motion(12, 0), "startup_ready",
                        "control_pose_and_fisheye_ready");
    }

    if (state.phase == 1) {
        if (!frame.pose) return hold(StageId::Stage1, state, "pose_not_ready");
        if (!state.pose_latched) latch_segment_pose(state, *frame.pose);
        const double progress = segment_progress(state, *frame.pose);
        if (progress >= kStoneDistance) {
            next_phase(StageId::Stage1, state, frame.now);
            return decision(StageId::Stage1, state, motion(12, 0), "stone_road_complete",
                            "pose_distance_reached");
        }
        if (progress < kStoneTailStart) {
            auto command = motion(62, 81, 0.235, 0.0, 0.0, 0.235, 500);
            command.step_height = {0.08F, 0.08F};
            command.rpy_des[1] = -0.01F;
            return decision(StageId::Stage1, state, command, "stone_road_user_gait",
                            "pose_distance_first_segment");
        }
        auto command = motion(11, 27, 0.12, 0.0, 0.0, 0.235, 500);
        command.rpy_des[1] = -0.01F;
        return decision(StageId::Stage1, state, command, "stone_road_tail",
                        "pose_distance_tail_segment");
    }

    if (state.phase == 2) {
        if (phase_seconds(state, frame) >= 0.5) next_phase(StageId::Stage1, state, frame.now);
        return decision(StageId::Stage1, state, motion(12, 0), "stone_road_end_stand",
                        "recovery_stand_settle");
    }

    auto align_tangent = [&](std::string action) -> BusinessDecision {
        if (!color_frame(frame.right_fisheye) && !frame.observations.right_yellow_line)
            return hold(StageId::Stage1, state, "right_fisheye_not_ready");
        const auto line = right_line(frame);
        if (!line.detected) {
            ++state.lost_frames;
            if (state.lost_frames >= 20 && config_.continue_after_soft_timeout) {
                if (state.phase == 3) next_phase(StageId::Stage1, state, frame.now);
                else enter_step(state, state.step + 1);
                auto result = decision(StageId::Stage1, state, motion(12, 0), action,
                                       "line_lost_soft_continue");
                result.degraded = true;
                return result;
            }
            return decision(StageId::Stage1, state, motion(11, 27, 0.0, 0.0, 0.08),
                            action, "right_yellow_tangent_search");
        }
        state.lost_frames = 0;
        state.stable_frames = std::abs(line.angle_deg) <= 6.0 ? state.stable_frames + 1 : 0;
        const double wz = std::clamp(-0.18 * line.angle_deg, -0.22, 0.22);
        if (state.stable_frames >= 16) {
            if (state.phase == 3) next_phase(StageId::Stage1, state, frame.now);
            else enter_step(state, state.step + 1);
        }
        return decision(StageId::Stage1, state, motion(11, 27, 0.0, 0.0, wz), action,
                        state.stable_frames > 0 ? "angle_stable" : "angle_correction");
    };

    if (state.phase == 3) return align_tangent("right_tangent_align");

    if (state.phase == 4) {
        if (state.step == 0) {
            if (!color_frame(frame.right_fisheye) && !frame.observations.right_bottom_yellow)
                return hold(StageId::Stage1, state, "right_fisheye_not_ready");
            const auto marker = right_marker(frame, 0.30);
            state.stable_frames = marker.detected ? state.stable_frames + 1 : 0;
            if (state.stable_frames >= 3) enter_step(state, 1);
            return decision(StageId::Stage1, state,
                            motion(11, 27, 0.0, marker.detected ? 0.0 : 0.05),
                            "pre_turn_right_roi_shift",
                            marker.detected ? "right_yellow_roi_stable" : "seek_right_yellow_roi");
        }
        if (!frame.pose) return hold(StageId::Stage1, state, "pose_not_ready_for_turn");
        if (!state.turn_target_latched) {
            state.target_yaw_deg = perception::normalize_angle_degrees(frame.pose->yaw_deg + 90.0);
            state.turn_target_latched = true;
        }
        const double error = perception::normalize_angle_degrees(state.target_yaw_deg - frame.pose->yaw_deg);
        state.stable_frames = std::abs(error) <= 3.0 ? state.stable_frames + 1 : 0;
        if (state.stable_frames >= 5) next_phase(StageId::Stage1, state, frame.now);
        return decision(StageId::Stage1, state,
                        motion(11, 27, 0.0, 0.0, std::clamp(0.04 * error, -0.35, 0.35)),
                        "relative_yaw_turn", "pose_yaw_closed_loop");
    }

    if (state.phase == 5) {
        if (state.step == 0) return align_tangent("post_turn_right_tangent_align");
        if (!color_frame(frame.right_fisheye) && !frame.observations.right_bottom_yellow)
            return hold(StageId::Stage1, state, "right_fisheye_not_ready");
        const auto marker = right_marker(frame, 0.45);
        state.stable_frames = marker.detected ? state.stable_frames + 1 : 0;
        if (state.stable_frames >= 3) next_phase(StageId::Stage1, state, frame.now);
        return decision(StageId::Stage1, state,
                        motion(11, 27, 0.0, marker.detected ? 0.0 : 0.05),
                        "post_turn_right_roi_shift",
                        marker.detected ? "second_right_yellow_roi_stable" : "seek_second_right_yellow_roi");
    }

    if (!frame.pose) return hold(StageId::Stage1, state, "pose_not_ready_for_exit");
    if (!state.pose_latched) latch_segment_pose(state, *frame.pose);
    const auto marker = left_marker(frame);
    if (marker.detected) state.stable_frames += 1;
    else state.stable_frames = 0;
    const double progress = segment_progress(state, *frame.pose);
    if (state.stable_frames >= 3)
        return complete(StageId::Stage1, state, "left_yellow_roi_filled");
    if (progress >= 0.70)
        return complete(StageId::Stage1, state, "max_forward_distance_reached");
    return decision(StageId::Stage1, state, motion(11, 27, 0.12), "post_turn_forward",
                    "seek_left_yellow_roi");
}

}  // namespace doogle::competition
