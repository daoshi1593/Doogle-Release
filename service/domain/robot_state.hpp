#pragma once

namespace doogle {

enum class Stage { Search, Approach, Kick };

struct ControlState {
    Stage stage{Stage::Search};
    int lost_ticks{0};
};

}  // namespace doogle
