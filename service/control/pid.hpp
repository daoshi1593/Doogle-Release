#pragma once

namespace doogle::control {

struct PidConfig {
    double kp{};
    double ki{};
    double kd{};
    double output_min{-1.0};
    double output_max{1.0};
};

struct PidState {
    double integral{};
    double previous_error{};
    bool initialized{false};
};

struct PidResult {
    PidState state;
    double output{};
};

[[nodiscard]] PidResult update_pid(const PidConfig&, const PidState&, double error, double dt);

}  // namespace doogle::control
