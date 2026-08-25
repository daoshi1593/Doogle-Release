#pragma once

#include "service/domain/robot_state.hpp"

namespace doogle::stage6 {

enum class StageId { Stage1, Stage2, Stage3, Stage4, Stage5, Stage6 };

struct StageState {
    StageId stage{StageId::Stage1};
    bool completed{false};
    int transition_count{0};
};

[[nodiscard]] StageState advance(StageState previous, bool stage_complete);

}  // namespace doogle::stage6
