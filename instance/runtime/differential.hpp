#pragma once

#include <iomanip>
#include <ostream>

#include "instance/competition/reducer.hpp"

namespace doogle::runtime {

inline void write_decision_json(std::ostream& output, const stage6::Decision& decision) {
    output << std::setprecision(9) << "{\"stage\":"
           << static_cast<int>(decision.next_state.stage)
           << ",\"lost_ticks\":" << decision.next_state.lost_ticks
           << ",\"mode\":" << static_cast<int>(decision.desired_motion.mode)
           << ",\"gait_id\":" << decision.desired_motion.gait_id
           << ",\"vx\":" << decision.desired_motion.vx
           << ",\"vy\":" << decision.desired_motion.vy
           << ",\"wz\":" << decision.desired_motion.wz
           << ",\"effects\":" << decision.effects.size() << '}';
}

}  // namespace doogle::runtime
