#pragma once

#include <chrono>

#include "service/domain/observation.hpp"
#include "instance/competition/reducer.hpp"
#include "instance/runtime/command_slot.hpp"

namespace doogle::runtime {

class ControlLoop {
public:
    explicit ControlLoop(stage6::Config config) : config_(config) {}

    [[nodiscard]] stage6::Decision tick(const Observation& observation) {
        const auto decision = stage6::reduce(config_, state_, observation);
        state_ = decision.next_state;
        command_slot_.store(decision.desired_motion);
        return decision;
    }

    [[nodiscard]] const ControlState& state() const { return state_; }
    [[nodiscard]] CommandSlot& command_slot() { return command_slot_; }

private:
    stage6::Config config_;
    ControlState state_{};
    CommandSlot command_slot_;
};

}  // namespace doogle::runtime
