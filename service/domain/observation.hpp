#pragma once

#include <chrono>
#include <optional>

namespace doogle {

using SteadyTime = std::chrono::steady_clock::time_point;

struct BallObservation {
    float x{0.0F};
    float y{0.0F};
    float distance{0.0F};
};

struct TofObservation { float distance{0.0F}; };

struct Observation {
    SteadyTime now{};
    std::optional<BallObservation> ball;
    std::optional<TofObservation> tof;
};

}  // namespace doogle
