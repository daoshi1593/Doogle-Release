#include "instance/competition/stages.hpp"

namespace doogle::stage6 {

StageState advance(StageState previous, bool stage_complete) {
    if (!stage_complete || previous.stage == StageId::Stage6) return previous;
    previous.stage = static_cast<StageId>(static_cast<int>(previous.stage) + 1);
    previous.completed = false;
    ++previous.transition_count;
    return previous;
}

}  // namespace doogle::stage6
