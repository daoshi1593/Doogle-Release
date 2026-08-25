#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <string_view>
#include <utility>

#include "instance/api/business_instance.hpp"
#include "instance/api/sensor_provider.hpp"
#include "instance/runtime/runtime_host.hpp"

namespace {

class BlueBallFollower final : public doogle::instance::IBusinessInstance {
public:
    [[nodiscard]] std::string_view name() const noexcept override {
        return "blue_ball_follower";
    }

    [[nodiscard]] doogle::competition::BusinessDecision tick(
        const doogle::competition::SensorFrame& frame) override {
        doogle::competition::BusinessDecision decision;
        decision.phase_key = "follow_blue_ball";
        decision.command.contact = 15;
        const auto& ball = frame.observations.blue_ball;
        if (!ball || !ball->component.detected) {
            decision.command.mode = 12;
            decision.fail_closed = true;
            decision.action = "stand";
            decision.reason = "blue_ball_not_ready";
            return decision;
        }

        const double error = ball->component.center_x_ratio - 0.5;
        decision.command.mode = 11;
        decision.command.gait_id = 27;
        decision.command.vel_des[0] = 0.08F;
        decision.command.vel_des[1] = static_cast<float>(-0.25 * error);
        decision.action = "follow";
        decision.reason = "blue_ball_observation";
        decision.stage_complete = std::abs(error) < 0.03;
        return decision;
    }

    void reset() override {}
};

class ExampleSensorProvider final : public doogle::instance::ISensorProvider {
public:
    [[nodiscard]] bool open() override {
        index_ = 0;
        return true;
    }

    void close() noexcept override {}

    [[nodiscard]] doogle::instance::SensorRead read(std::chrono::milliseconds) override {
        constexpr std::array<double, 3> kCenters{0.25, 0.40, 0.50};
        if (index_ >= kCenters.size()) {
            return doogle::instance::SensorRead::end_of_stream();
        }
        doogle::competition::SensorFrame frame;
        doogle::perception::ColorDetection detection;
        detection.target = doogle::perception::VisualTarget::BlueBall;
        detection.component.detected = true;
        detection.component.center_x_ratio = kCenters[index_++];
        detection.confidence = 1.0;
        frame.observations.blue_ball = detection;
        return doogle::instance::SensorRead::ready(std::move(frame));
    }

private:
    std::size_t index_{};
};

class ExampleMotionSink final : public doogle::ports::MotionSink {
public:
    void publish(const doogle::protocol::RobotControlCommand& command) override {
        last_command_ = command;
        ++count_;
    }

    [[nodiscard]] std::size_t count() const noexcept { return count_; }
    [[nodiscard]] const doogle::protocol::RobotControlCommand& last_command() const noexcept {
        return last_command_;
    }

private:
    std::size_t count_{};
    doogle::protocol::RobotControlCommand last_command_{};
};

}  // namespace

int main() {
    BlueBallFollower business;
    ExampleSensorProvider sensors;
    ExampleMotionSink motion;
    doogle::runtime::RuntimeHostConfig config;
    config.period = std::chrono::milliseconds{0};
    config.max_ticks = 8;
    config.allow_incomplete = false;
    doogle::runtime::RuntimeHost host{business, sensors, motion, config};
    const auto report = host.run();
    return report.success && motion.count() == 4 && motion.last_command().mode == 12 ? 0 : 1;
}
