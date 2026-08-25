#pragma once

#include <vector>

#include "service/domain/effects.hpp"
#include "service/domain/motion_intent.hpp"
#include "service/domain/observation.hpp"
#include "service/domain/robot_state.hpp"

namespace doogle::stage6 {

struct Config {
    int approach_gait{27};
    float approach_vx{0.08F};
    float approach_vy_limit{0.08F};
    float kick_distance{0.25F};
    int max_lost_ticks{10};
};

struct Decision {
    ControlState next_state;
    MotionIntent desired_motion;
    std::vector<Effect> effects;
};

[[nodiscard]] Decision reduce(const Config&, const ControlState&, const Observation&);

}  // namespace doogle::stage6
