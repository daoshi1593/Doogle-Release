#pragma once

#include <fstream>
#include <optional>
#include <string>

#include "service/domain/observation.hpp"
#include "instance/competition/reducer.hpp"

namespace doogle::runtime {

class ObservationRecorder {
public:
    explicit ObservationRecorder(const std::string& path) : output_(path) {}
    void append(const Observation& observation) {
        output_ << observation.now.time_since_epoch().count() << ' '
                << (observation.ball ? 1 : 0) << ' ';
        if (observation.ball) {
            output_ << observation.ball->x << ' ' << observation.ball->y << ' '
                    << observation.ball->distance;
        } else {
            output_ << "0 0 0";
        }
        output_ << '\n';
    }

private:
    std::ofstream output_;
};

class ObservationReplay {
public:
    explicit ObservationReplay(const std::string& path) : input_(path) {}
    [[nodiscard]] std::optional<Observation> next() {
        long long timestamp{};
        int has_ball{};
        BallObservation ball{};
        if (!(input_ >> timestamp >> has_ball >> ball.x >> ball.y >> ball.distance)) return std::nullopt;
        Observation observation;
        observation.now = SteadyTime{std::chrono::steady_clock::duration{timestamp}};
        if (has_ball != 0) observation.ball = ball;
        return observation;
    }

private:
    std::ifstream input_;
};

}  // namespace doogle::runtime
