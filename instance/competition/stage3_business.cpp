#include "instance/competition/business_engine.hpp"

#include <algorithm>
#include <cmath>

namespace doogle::competition {

BusinessDecision BusinessEngine::stage3(const SensorFrame& frame, StageState& state) {
    if (state.phase == 0) {
        if (phase_seconds(state, frame) >= 0.5) next_phase(StageId::Stage3, state, frame.now);
        return decision(StageId::Stage3, state, motion(12, 0), "startup_stand",
                        "recovery_stand_settle");
    }

    if (state.phase == 1) {
        if (frame.left_fisheye.empty() || frame.right_fisheye.empty())
            return hold(StageId::Stage3, state, "stereo_fisheye_not_ready");
        next_phase(StageId::Stage3, state, frame.now);
        return decision(StageId::Stage3, state, motion(12, 0), "fisheye_ready",
                        "left_and_right_first_frame_ready");
    }

    const auto left = frame.observations.left_yellow_line
                          ? *frame.observations.left_yellow_line
                          : perception::analyze_yellow_line(frame.left_fisheye);
    const auto right = frame.observations.right_yellow_line
                           ? *frame.observations.right_yellow_line
                           : perception::analyze_yellow_line(frame.right_fisheye);

    if (state.phase == 2) {
        const auto correction = perception::compute_s_curve_command(left, right, 0.15);
        if (!left.detected && !right.detected) ++state.lost_frames;
        else state.lost_frames = 0;
        if (state.lost_frames >= 5 && phase_seconds(state, frame) >= 1.0)
            next_phase(StageId::Stage3, state, frame.now);
        return decision(StageId::Stage3, state,
                        motion(11, 27, correction.vx, correction.vy, correction.wz),
                        correction.hold_yaw ? "s_curve_hold_yaw" : "s_curve_closed_loop",
                        left.detected || right.detected ? "yellow_tangent_control"
                                                       : "left_exit_region_reached");
    }

    if (!right.detected) {
        ++state.lost_frames;
        return decision(StageId::Stage3, state, motion(11, 27, 0.03),
                        "right_horizontal_search", "right_line_missing");
    }
    state.lost_frames = 0;
    state.stable_frames = std::abs(right.angle_deg) <= 3.0 ? state.stable_frames + 1 : 0;
    if (state.stable_frames >= 16)
        return complete(StageId::Stage3, state, "right_fisheye_horizontal_aligned");
    return decision(StageId::Stage3, state,
                    motion(11, 27, 0.0, 0.0,
                           std::clamp(-0.18 * right.angle_deg, -0.22, 0.22)),
                    "right_horizontal_align", "right_fisheye_angle_closed_loop");
}

}  // namespace doogle::competition
