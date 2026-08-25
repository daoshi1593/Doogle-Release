#include "instance/competition/reducer.hpp"

#include <algorithm>

namespace doogle::stage6 {

Decision reduce(const Config& config, const ControlState& previous, const Observation& observation) {
    Decision decision{previous, {}, {}};
    if (!observation.ball) {
        decision.next_state.lost_ticks = previous.lost_ticks + 1;
        decision.desired_motion = {MotionMode::Walk, config.approach_gait, 15, 0.0F,
                                   0.04F, 0.0F, 0.0F};
        if (decision.next_state.lost_ticks > config.max_lost_ticks) {
            decision.next_state.stage = Stage::Search;
        }
        return decision;
    }

    decision.next_state.lost_ticks = 0;
    const auto& ball = *observation.ball;
    if (ball.distance <= config.kick_distance) {
        decision.next_state.stage = Stage::Kick;
        decision.desired_motion = {MotionMode::Stop, 0, 0, 0.0F, 0.0F, 0.0F, 0.0F};
        decision.effects.emplace_back(PlayAudio{AudioId::BallReached});
        return decision;
    }

    decision.next_state.stage = Stage::Approach;
    const float vy = std::clamp(ball.x, -config.approach_vy_limit, config.approach_vy_limit);
    decision.desired_motion = {MotionMode::Walk, config.approach_gait, 15,
                               config.approach_vx, vy, ball.y, 0.0F};
    return decision;
}

}  // namespace doogle::stage6
