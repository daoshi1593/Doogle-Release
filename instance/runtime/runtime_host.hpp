#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "instance/api/business_instance.hpp"
#include "instance/api/sensor_provider.hpp"
#include "service/port/motion_sink.hpp"

namespace doogle::runtime {

enum class RuntimeExit {
    Completed,
    TickBudgetExhausted,
    SensorOpenFailed,
    SensorEndOfStream,
    SensorError,
    BusinessError,
    OutputError,
    StopRequested,
    InvalidConfiguration,
};

[[nodiscard]] std::string_view runtime_exit_name(RuntimeExit value) noexcept;

struct RuntimeHostConfig {
    std::chrono::milliseconds period{50};
    std::size_t max_ticks{1};
    bool stop_on_complete{true};
    bool allow_incomplete{true};
    bool stop_on_exit{true};
    bool reset_business_on_start{true};
};

struct RuntimeReport {
    RuntimeExit exit{RuntimeExit::InvalidConfiguration};
    bool success{};
    std::size_t ticks{};
    std::size_t commands_published{};
    std::optional<competition::BusinessDecision> last_decision;
    std::string detail;
};

class RuntimeHost {
public:
    RuntimeHost(instance::IBusinessInstance& business,
                instance::ISensorProvider& sensors,
                ports::MotionSink& motion,
                RuntimeHostConfig config = {});

    [[nodiscard]] RuntimeReport run();
    void request_stop() noexcept;
    [[nodiscard]] bool stop_requested() const noexcept;

private:
    [[nodiscard]] bool publish(protocol::RobotControlCommand command,
                               RuntimeReport& report) noexcept;
    void publish_stop(RuntimeReport& report) noexcept;
    [[nodiscard]] RuntimeReport finish(RuntimeReport report, bool sensor_open) noexcept;

    instance::IBusinessInstance& business_;
    instance::ISensorProvider& sensors_;
    ports::MotionSink& motion_;
    RuntimeHostConfig config_;
    std::atomic_bool stop_requested_{};
    std::uint8_t life_count_{};
};

}  // namespace doogle::runtime
