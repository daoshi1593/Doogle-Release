#include <cassert>
#include <array>
#include <chrono>
#include <cstddef>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "instance/api/business_instance.hpp"
#include "instance/api/sensor_provider.hpp"
#include "instance/api/stage_business_instance.hpp"
#include "instance/runtime/runtime_host.hpp"
#include "instance/runtime/standard_sensor_provider.hpp"

namespace {

using doogle::competition::BusinessDecision;
using doogle::competition::SensorFrame;
using doogle::instance::SensorRead;
using doogle::runtime::RuntimeExit;
using doogle::runtime::RuntimeHost;
using doogle::runtime::RuntimeHostConfig;

SensorFrame ready_frame() {
    SensorFrame frame;
    frame.control_safe = true;
    return frame;
}

class FakeBusiness final : public doogle::instance::IBusinessInstance {
public:
    [[nodiscard]] std::string_view name() const noexcept override { return "fake_business"; }

    [[nodiscard]] BusinessDecision tick(const SensorFrame& frame) override {
        if (throw_on_tick) throw std::runtime_error{"tick_failure"};
        last_frame = frame;
        ++tick_calls;
        BusinessDecision decision;
        decision.command.mode = 11;
        decision.command.gait_id = 27;
        decision.command.contact = 15;
        decision.command.vel_des = {0.11F, -0.07F, 0.03F};
        decision.command.rpy_des = {0.1F, 0.2F, 0.3F};
        decision.command.step_height = {0.04F, 0.05F};
        decision.command.duration = 900;
        decision.action = "fake_action";
        decision.reason = "fake_reason";
        decision.stage_complete = complete_after > 0 && tick_calls >= complete_after;
        return decision;
    }

    void reset() override {
        if (throw_on_reset) throw std::runtime_error{"reset_failure"};
        ++reset_calls;
        tick_calls = 0;
    }

    int reset_calls{};
    int tick_calls{};
    int complete_after{};
    bool throw_on_reset{};
    bool throw_on_tick{};
    SensorFrame last_frame{};
};

class FakeSensorProvider final : public doogle::instance::ISensorProvider {
public:
    [[nodiscard]] bool open() override {
        ++open_calls;
        if (throw_on_open) throw std::runtime_error{"open_failure"};
        return open_result;
    }

    void close() noexcept override { ++close_calls; }

    [[nodiscard]] SensorRead read(std::chrono::milliseconds timeout) override {
        ++read_calls;
        timeouts.push_back(timeout);
        if (on_read) on_read();
        if (throw_on_read) throw std::runtime_error{"read_failure"};
        if (next_read < reads.size()) return std::move(reads[next_read++]);
        if (repeat_ready) return SensorRead::ready(ready_frame());
        return SensorRead::end_of_stream();
    }

    bool open_result{true};
    bool throw_on_open{};
    bool throw_on_read{};
    bool repeat_ready{};
    int open_calls{};
    int close_calls{};
    int read_calls{};
    std::size_t next_read{};
    std::vector<SensorRead> reads;
    std::vector<std::chrono::milliseconds> timeouts;
    std::function<void()> on_read;
};

class RecordingSink final : public doogle::ports::MotionSink {
public:
    void publish(const doogle::protocol::RobotControlCommand& command) override {
        ++publish_attempts;
        if (throw_on_publish) throw std::runtime_error{"publish_failure"};
        commands.push_back(command);
    }

    bool throw_on_publish{};
    int publish_attempts{};
    std::vector<doogle::protocol::RobotControlCommand> commands;
};

RuntimeHostConfig config_for(std::size_t ticks = 1) {
    RuntimeHostConfig config;
    config.period = std::chrono::milliseconds{0};
    config.max_ticks = ticks;
    return config;
}

void test_exit_names_are_stable() {
    assert(doogle::runtime::runtime_exit_name(RuntimeExit::Completed) == "completed");
    assert(doogle::runtime::runtime_exit_name(RuntimeExit::TickBudgetExhausted) ==
           "tick_budget_exhausted");
    assert(doogle::runtime::runtime_exit_name(RuntimeExit::SensorOpenFailed) ==
           "sensor_open_failed");
    assert(doogle::runtime::runtime_exit_name(RuntimeExit::SensorEndOfStream) ==
           "sensor_end_of_stream");
    assert(doogle::runtime::runtime_exit_name(RuntimeExit::SensorError) == "sensor_error");
    assert(doogle::runtime::runtime_exit_name(RuntimeExit::BusinessError) == "business_error");
    assert(doogle::runtime::runtime_exit_name(RuntimeExit::OutputError) == "output_error");
    assert(doogle::runtime::runtime_exit_name(RuntimeExit::StopRequested) == "stop_requested");
    assert(doogle::runtime::runtime_exit_name(RuntimeExit::InvalidConfiguration) ==
           "invalid_configuration");
}

void test_sensor_read_factories() {
    auto frame = ready_frame();
    const auto ready = SensorRead::ready(std::move(frame));
    assert(ready.status == doogle::instance::SensorReadStatus::Ready);
    assert(ready.frame.control_safe);
    const auto waiting = SensorRead::not_ready("waiting");
    assert(waiting.status == doogle::instance::SensorReadStatus::NotReady);
    assert(waiting.detail == "waiting");
    assert(SensorRead::end_of_stream().status ==
           doogle::instance::SensorReadStatus::EndOfStream);
    assert(SensorRead::error().status == doogle::instance::SensorReadStatus::Error);
}

void test_standard_provider_rejects_read_before_open() {
    doogle::runtime::StandardSensorProvider sensors;
    const auto input = sensors.read(std::chrono::milliseconds{0});
    assert(input.status == doogle::instance::SensorReadStatus::Error);
    assert(input.detail == "standard_sensor_provider_not_open");
}

void test_standard_provider_empty_input_is_not_ready() {
    doogle::runtime::StandardSensorProvider sensors;
    assert(sensors.open());
    const auto input = sensors.read(std::chrono::milliseconds{0});
    assert(input.status == doogle::instance::SensorReadStatus::NotReady);
    assert(input.detail == "standard_sensor_waiting_for_input");
    sensors.close();
    assert(sensors.read(std::chrono::milliseconds{0}).status ==
           doogle::instance::SensorReadStatus::Error);
}

void test_standard_provider_can_allow_empty_input() {
    doogle::runtime::StandardSensorProviderConfig config;
    config.allow_empty_frames = true;
    doogle::runtime::StandardSensorProvider sensors{config};
    assert(sensors.config().allow_empty_frames);
    assert(sensors.open());
    const auto input = sensors.read(std::chrono::milliseconds{0});
    assert(input.status == doogle::instance::SensorReadStatus::Ready);
    assert(input.frame.rgb.empty() && !input.frame.pose.has_value());
}

void test_standard_provider_invalid_file_fails_open() {
    doogle::runtime::StandardSensorProviderConfig config;
    config.rgb_path = "missing-doogle-image.png";
    doogle::runtime::StandardSensorProvider sensors{std::move(config)};
    assert(!sensors.open());
}

void test_standard_provider_is_movable() {
    doogle::runtime::StandardSensorProvider sensors;
    assert(sensors.open());
    doogle::runtime::StandardSensorProvider moved{std::move(sensors)};
    const auto input = moved.read(std::chrono::milliseconds{0});
    assert(input.status == doogle::instance::SensorReadStatus::NotReady);
}

void test_standard_provider_composes_with_runtime_host() {
    FakeBusiness business;
    doogle::runtime::StandardSensorProvider sensors;
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for()};
    const auto report = host.run();
    assert(report.success && report.ticks == 1);
    assert(report.last_decision->fail_closed);
    assert(report.last_decision->reason == "standard_sensor_waiting_for_input");
    assert(motion.commands.front().mode == 12);
}

void test_stage_business_adapter() {
    doogle::competition::BusinessEngineConfig engine_config;
    engine_config.start_phase[0] = 1;
    doogle::instance::StageBusinessInstance business{
        doogle::competition::StageId::Stage1, engine_config};
    assert(business.name() == "stage1");
    assert(business.stage() == doogle::competition::StageId::Stage1);
    const auto decision = business.tick(ready_frame());
    assert(decision.stage == doogle::competition::StageId::Stage1);
    assert(decision.fail_closed);
    business.reset();
    const auto after_reset = business.tick(ready_frame());
    assert(after_reset.phase == decision.phase);
}

void test_invalid_zero_tick_configuration() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    RecordingSink motion;
    auto config = config_for(0);
    RuntimeHost host{business, sensors, motion, config};
    const auto report = host.run();
    assert(report.exit == RuntimeExit::InvalidConfiguration && !report.success);
    assert(sensors.open_calls == 0 && sensors.close_calls == 0);
    assert(motion.commands.size() == 1 && motion.commands.back().mode == 12);
}

void test_invalid_negative_period_configuration() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    RecordingSink motion;
    auto config = config_for();
    config.period = std::chrono::milliseconds{-1};
    RuntimeHost host{business, sensors, motion, config};
    const auto report = host.run();
    assert(report.exit == RuntimeExit::InvalidConfiguration);
    assert(sensors.open_calls == 0);
}

void test_stop_requested_before_open() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for()};
    host.request_stop();
    assert(host.stop_requested());
    const auto report = host.run();
    assert(report.exit == RuntimeExit::StopRequested && report.success);
    assert(sensors.open_calls == 0 && business.reset_calls == 0);
}

void test_sensor_open_failure() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.open_result = false;
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for()};
    const auto report = host.run();
    assert(report.exit == RuntimeExit::SensorOpenFailed && !report.success);
    assert(sensors.open_calls == 1 && sensors.close_calls == 0);
    assert(business.reset_calls == 0);
}

void test_sensor_open_exception() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.throw_on_open = true;
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for()};
    const auto report = host.run();
    assert(report.exit == RuntimeExit::SensorOpenFailed);
    assert(sensors.close_calls == 0);
}

void test_business_reset_default_and_provider_close() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.reads.push_back(SensorRead::ready(ready_frame()));
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for()};
    const auto report = host.run();
    assert(report.exit == RuntimeExit::TickBudgetExhausted && report.success);
    assert(business.reset_calls == 1 && business.tick_calls == 1);
    assert(sensors.open_calls == 1 && sensors.close_calls == 1);
}

void test_business_reset_can_be_disabled() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.reads.push_back(SensorRead::ready(ready_frame()));
    RecordingSink motion;
    auto config = config_for();
    config.reset_business_on_start = false;
    RuntimeHost host{business, sensors, motion, config};
    static_cast<void>(host.run());
    assert(business.reset_calls == 0 && business.tick_calls == 1);
}

void test_business_reset_exception() {
    FakeBusiness business;
    business.throw_on_reset = true;
    FakeSensorProvider sensors;
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for()};
    const auto report = host.run();
    assert(report.exit == RuntimeExit::BusinessError && report.detail == "business_reset_failed");
    assert(sensors.close_calls == 1 && business.tick_calls == 0);
}

void test_sensor_end_of_stream() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.reads.push_back(SensorRead::end_of_stream("finished_input"));
    RecordingSink motion;
    auto config = config_for();
    config.allow_incomplete = false;
    RuntimeHost host{business, sensors, motion, config};
    const auto report = host.run();
    assert(report.exit == RuntimeExit::SensorEndOfStream && !report.success);
    assert(report.detail == "finished_input" && report.ticks == 0);
    assert(business.tick_calls == 0 && sensors.close_calls == 1);
}

void test_sensor_error_result() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.reads.push_back(SensorRead::error("decode_failed"));
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for()};
    const auto report = host.run();
    assert(report.exit == RuntimeExit::SensorError && report.detail == "decode_failed");
    assert(report.ticks == 0 && motion.commands.back().mode == 12);
}

void test_sensor_read_exception() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.throw_on_read = true;
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for()};
    const auto report = host.run();
    assert(report.exit == RuntimeExit::SensorError && report.detail == "read_failure");
    assert(sensors.close_calls == 1);
}

void test_business_tick_exception() {
    FakeBusiness business;
    business.throw_on_tick = true;
    FakeSensorProvider sensors;
    sensors.reads.push_back(SensorRead::ready(ready_frame()));
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for()};
    const auto report = host.run();
    assert(report.exit == RuntimeExit::BusinessError && report.detail == "tick_failure");
    assert(report.ticks == 0 && sensors.close_calls == 1);
}

void test_motion_output_exception() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.reads.push_back(SensorRead::ready(ready_frame()));
    RecordingSink motion;
    motion.throw_on_publish = true;
    RuntimeHost host{business, sensors, motion, config_for()};
    const auto report = host.run();
    assert(report.exit == RuntimeExit::OutputError && report.detail == "motion_publish_failed");
    assert(report.commands_published == 0 && motion.publish_attempts == 2);
}

void test_not_ready_forces_fail_closed_stop() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.reads.push_back(SensorRead::not_ready("camera_waiting"));
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for()};
    const auto report = host.run();
    assert(report.last_decision.has_value());
    assert(report.last_decision->fail_closed);
    assert(report.last_decision->action == "runtime_hold");
    assert(report.last_decision->reason == "camera_waiting");
    assert(!business.last_frame.control_safe);
    assert(motion.commands.front().mode == 12 && motion.commands.front().contact == 15);
}

void test_unsafe_ready_frame_forces_stop() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    auto frame = ready_frame();
    frame.control_safe = false;
    sensors.reads.push_back(SensorRead::ready(std::move(frame)));
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for()};
    const auto report = host.run();
    assert(report.last_decision->fail_closed);
    assert(motion.commands.front().mode == 12);
}

void test_frame_stop_request_forces_stop() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    auto frame = ready_frame();
    frame.stop_requested = true;
    sensors.reads.push_back(SensorRead::ready(std::move(frame)));
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for()};
    const auto report = host.run();
    assert(report.last_decision->fail_closed);
    assert(motion.commands.front().mode == 12);
}

void test_ready_command_is_preserved() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.reads.push_back(SensorRead::ready(ready_frame()));
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for()};
    const auto report = host.run();
    assert(report.commands_published == 2);
    const auto& command = motion.commands.front();
    const std::array<float, 3> expected_velocity{0.11F, -0.07F, 0.03F};
    const std::array<float, 3> expected_rpy{0.1F, 0.2F, 0.3F};
    const std::array<float, 2> expected_step_height{0.04F, 0.05F};
    assert(command.mode == 11 && command.gait_id == 27 && command.contact == 15);
    assert(command.vel_des == expected_velocity);
    assert(command.rpy_des == expected_rpy);
    assert(command.step_height == expected_step_height);
    assert(command.duration == 900 && command.life_count == 1);
    assert(motion.commands.back().mode == 12 && motion.commands.back().life_count == 2);
}

void test_complete_stops_early() {
    FakeBusiness business;
    business.complete_after = 1;
    FakeSensorProvider sensors;
    sensors.repeat_ready = true;
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for(5)};
    const auto report = host.run();
    assert(report.exit == RuntimeExit::Completed && report.success);
    assert(report.ticks == 1 && sensors.read_calls == 1);
    assert(motion.commands.size() == 2 && motion.commands.back().mode == 12);
}

void test_strict_tick_budget_is_failure() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.repeat_ready = true;
    RecordingSink motion;
    auto config = config_for(3);
    config.allow_incomplete = false;
    RuntimeHost host{business, sensors, motion, config};
    const auto report = host.run();
    assert(report.exit == RuntimeExit::TickBudgetExhausted && !report.success);
    assert(report.ticks == 3 && business.tick_calls == 3);
}

void test_permissive_tick_budget_is_success() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.repeat_ready = true;
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for(2)};
    const auto report = host.run();
    assert(report.exit == RuntimeExit::TickBudgetExhausted && report.success);
    assert(report.ticks == 2);
}

void test_stop_on_exit_can_be_disabled() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.reads.push_back(SensorRead::ready(ready_frame()));
    RecordingSink motion;
    auto config = config_for();
    config.stop_on_exit = false;
    RuntimeHost host{business, sensors, motion, config};
    const auto report = host.run();
    assert(report.commands_published == 1 && motion.commands.size() == 1);
    assert(motion.commands.front().mode == 11);
}

void test_complete_can_continue_to_budget() {
    FakeBusiness business;
    business.complete_after = 1;
    FakeSensorProvider sensors;
    sensors.repeat_ready = true;
    RecordingSink motion;
    auto config = config_for(2);
    config.stop_on_complete = false;
    config.stop_on_exit = false;
    RuntimeHost host{business, sensors, motion, config};
    const auto report = host.run();
    assert(report.exit == RuntimeExit::TickBudgetExhausted);
    assert(report.ticks == 2 && business.tick_calls == 2);
}

void test_provider_timeout_contract() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.repeat_ready = true;
    RecordingSink motion;
    auto config = config_for(2);
    config.period = std::chrono::milliseconds{17};
    config.stop_on_exit = false;
    RuntimeHost host{business, sensors, motion, config};
    static_cast<void>(host.run());
    assert(sensors.timeouts.size() == 2);
    assert(sensors.timeouts[0] == std::chrono::milliseconds{17});
    assert(sensors.timeouts[1] == std::chrono::milliseconds{0});
}

void test_provider_timestamp_is_preserved() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    auto frame = ready_frame();
    const auto timestamp = std::chrono::steady_clock::time_point{std::chrono::milliseconds{42}};
    frame.now = timestamp;
    sensors.reads.push_back(SensorRead::ready(std::move(frame)));
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for()};
    static_cast<void>(host.run());
    assert(business.last_frame.now == timestamp);
}

void test_host_can_run_again_with_fresh_heartbeat() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.repeat_ready = true;
    RecordingSink motion;
    auto config = config_for();
    config.stop_on_exit = false;
    RuntimeHost host{business, sensors, motion, config};
    const auto first = host.run();
    const auto second = host.run();
    assert(first.success && second.success);
    assert(sensors.open_calls == 2 && sensors.close_calls == 2);
    assert(business.reset_calls == 2);
    assert(motion.commands.size() == 2);
    assert(motion.commands[0].life_count == 1 && motion.commands[1].life_count == 1);
}

void test_heartbeat_wraps_without_losing_command() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.repeat_ready = true;
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for(130)};
    const auto report = host.run();
    assert(report.commands_published == 131 && motion.commands.size() == 131);
    assert(motion.commands[0].life_count == 1);
    assert(motion.commands[127].life_count == 0);
    assert(motion.commands[128].life_count == 1);
    assert(motion.commands[129].life_count == 2);
    assert(motion.commands[129].duration == 900);
    assert(motion.commands[130].mode == 12 && motion.commands[130].life_count == 3);
}

void test_stop_request_during_read_is_enforced() {
    FakeBusiness business;
    FakeSensorProvider sensors;
    sensors.repeat_ready = true;
    RecordingSink motion;
    RuntimeHost host{business, sensors, motion, config_for(3)};
    sensors.on_read = [&host] { host.request_stop(); };
    const auto report = host.run();
    assert(report.exit == RuntimeExit::StopRequested && report.success);
    assert(report.ticks == 1 && report.last_decision->fail_closed);
    assert(motion.commands.front().mode == 12 && motion.commands.back().mode == 12);
}

}  // namespace

int main() {
    test_exit_names_are_stable();
    test_sensor_read_factories();
    test_standard_provider_rejects_read_before_open();
    test_standard_provider_empty_input_is_not_ready();
    test_standard_provider_can_allow_empty_input();
    test_standard_provider_invalid_file_fails_open();
    test_standard_provider_is_movable();
    test_standard_provider_composes_with_runtime_host();
    test_stage_business_adapter();
    test_invalid_zero_tick_configuration();
    test_invalid_negative_period_configuration();
    test_stop_requested_before_open();
    test_sensor_open_failure();
    test_sensor_open_exception();
    test_business_reset_default_and_provider_close();
    test_business_reset_can_be_disabled();
    test_business_reset_exception();
    test_sensor_end_of_stream();
    test_sensor_error_result();
    test_sensor_read_exception();
    test_business_tick_exception();
    test_motion_output_exception();
    test_not_ready_forces_fail_closed_stop();
    test_unsafe_ready_frame_forces_stop();
    test_frame_stop_request_forces_stop();
    test_ready_command_is_preserved();
    test_complete_stops_early();
    test_strict_tick_budget_is_failure();
    test_permissive_tick_budget_is_success();
    test_stop_on_exit_can_be_disabled();
    test_complete_can_continue_to_budget();
    test_provider_timeout_contract();
    test_provider_timestamp_is_preserved();
    test_host_can_run_again_with_fresh_heartbeat();
    test_heartbeat_wraps_without_losing_command();
    test_stop_request_during_read_is_enforced();
    return 0;
}
