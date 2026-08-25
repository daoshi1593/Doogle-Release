#include "instance/runtime/runtime_host.hpp"

#include <exception>
#include <thread>
#include <utility>

namespace doogle::runtime {
namespace {

protocol::RobotControlCommand stop_command() {
    protocol::RobotControlCommand command;
    command.mode = 12;
    command.contact = 15;
    return command;
}

}  // namespace

std::string_view runtime_exit_name(RuntimeExit value) noexcept {
    switch (value) {
        case RuntimeExit::Completed: return "completed";
        case RuntimeExit::TickBudgetExhausted: return "tick_budget_exhausted";
        case RuntimeExit::SensorOpenFailed: return "sensor_open_failed";
        case RuntimeExit::SensorEndOfStream: return "sensor_end_of_stream";
        case RuntimeExit::SensorError: return "sensor_error";
        case RuntimeExit::BusinessError: return "business_error";
        case RuntimeExit::OutputError: return "output_error";
        case RuntimeExit::StopRequested: return "stop_requested";
        case RuntimeExit::InvalidConfiguration: return "invalid_configuration";
    }
    return "unknown";
}

RuntimeHost::RuntimeHost(instance::IBusinessInstance& business,
                         instance::ISensorProvider& sensors,
                         ports::MotionSink& motion,
                         RuntimeHostConfig config)
    : business_(business), sensors_(sensors), motion_(motion), config_(config) {}

bool RuntimeHost::publish(protocol::RobotControlCommand command,
                          RuntimeReport& report) noexcept {
    life_count_ = static_cast<std::uint8_t>((life_count_ + 1U) % 128U);
    command.life_count = static_cast<std::int8_t>(life_count_);
    try {
        motion_.publish(command);
        ++report.commands_published;
        return true;
    } catch (...) {
        return false;
    }
}

void RuntimeHost::publish_stop(RuntimeReport& report) noexcept {
    if (!config_.stop_on_exit) return;
    static_cast<void>(publish(stop_command(), report));
}

RuntimeReport RuntimeHost::finish(RuntimeReport report, bool sensor_open) noexcept {
    publish_stop(report);
    if (sensor_open) sensors_.close();
    return report;
}

RuntimeReport RuntimeHost::run() {
    RuntimeReport report;
    life_count_ = 0;
    if (config_.max_ticks == 0 || config_.period.count() < 0) {
        report.exit = RuntimeExit::InvalidConfiguration;
        report.detail = "period_must_be_non_negative_and_max_ticks_positive";
        return finish(std::move(report), false);
    }
    if (stop_requested()) {
        report.exit = RuntimeExit::StopRequested;
        report.success = true;
        report.detail = "stop_requested_before_open";
        return finish(std::move(report), false);
    }

    bool sensor_open{};
    try {
        sensor_open = sensors_.open();
    } catch (...) {
        sensor_open = false;
    }
    if (!sensor_open) {
        report.exit = RuntimeExit::SensorOpenFailed;
        report.detail = "sensor_provider_open_failed";
        return finish(std::move(report), false);
    }

    if (config_.reset_business_on_start) {
        try {
            business_.reset();
        } catch (...) {
            report.exit = RuntimeExit::BusinessError;
            report.detail = "business_reset_failed";
            return finish(std::move(report), true);
        }
    }

    for (std::size_t tick = 0; tick < config_.max_ticks; ++tick) {
        if (stop_requested()) {
            report.exit = RuntimeExit::StopRequested;
            report.success = true;
            report.detail = "stop_requested";
            return finish(std::move(report), true);
        }

        instance::SensorRead input;
        try {
            input = sensors_.read(tick == 0 ? config_.period : std::chrono::milliseconds{0});
        } catch (const std::exception& error) {
            report.exit = RuntimeExit::SensorError;
            report.detail = error.what();
            return finish(std::move(report), true);
        } catch (...) {
            report.exit = RuntimeExit::SensorError;
            report.detail = "sensor_provider_threw";
            return finish(std::move(report), true);
        }

        if (input.status == instance::SensorReadStatus::EndOfStream) {
            report.exit = RuntimeExit::SensorEndOfStream;
            report.success = config_.allow_incomplete;
            report.detail = std::move(input.detail);
            return finish(std::move(report), true);
        }
        if (input.status == instance::SensorReadStatus::Error) {
            report.exit = RuntimeExit::SensorError;
            report.detail = std::move(input.detail);
            return finish(std::move(report), true);
        }

        if (input.status == instance::SensorReadStatus::NotReady) {
            input.frame.control_safe = false;
        }
        input.frame.stop_requested = input.frame.stop_requested || stop_requested();

        competition::BusinessDecision decision;
        try {
            decision = business_.tick(input.frame);
        } catch (const std::exception& error) {
            report.exit = RuntimeExit::BusinessError;
            report.detail = error.what();
            return finish(std::move(report), true);
        } catch (...) {
            report.exit = RuntimeExit::BusinessError;
            report.detail = "business_tick_threw";
            return finish(std::move(report), true);
        }

        if (input.status == instance::SensorReadStatus::NotReady ||
            !input.frame.control_safe || input.frame.stop_requested) {
            decision.command = stop_command();
            decision.fail_closed = true;
            decision.action = "runtime_hold";
            decision.reason = input.detail.empty() ? "sensor_not_ready" : std::move(input.detail);
        }
        ++report.ticks;
        report.last_decision = decision;
        if (!publish(decision.command, report)) {
            report.exit = RuntimeExit::OutputError;
            report.detail = "motion_publish_failed";
            return finish(std::move(report), true);
        }
        if (decision.stage_complete && config_.stop_on_complete) {
            report.exit = RuntimeExit::Completed;
            report.success = true;
            report.detail = "business_complete";
            return finish(std::move(report), true);
        }
        if (tick + 1 < config_.max_ticks && config_.period.count() > 0) {
            std::this_thread::sleep_for(config_.period);
        }
    }

    report.exit = RuntimeExit::TickBudgetExhausted;
    report.success = config_.allow_incomplete;
    report.detail = "tick_budget_exhausted";
    return finish(std::move(report), true);
}

void RuntimeHost::request_stop() noexcept {
    stop_requested_.store(true, std::memory_order_relaxed);
}

bool RuntimeHost::stop_requested() const noexcept {
    return stop_requested_.load(std::memory_order_relaxed);
}

}  // namespace doogle::runtime
