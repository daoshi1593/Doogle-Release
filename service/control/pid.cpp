#include "service/control/pid.hpp"

#include <algorithm>

namespace doogle::control {

PidResult update_pid(const PidConfig& config, const PidState& previous, double error, double dt) {
    PidState next = previous;
    const double safe_dt = std::max(dt, 0.0);
    next.integral += error * safe_dt;
    const double derivative = next.initialized && safe_dt > 0.0
                                  ? (error - previous.previous_error) / safe_dt
                                  : 0.0;
    next.previous_error = error;
    next.initialized = true;
    const double raw = config.kp * error + config.ki * next.integral + config.kd * derivative;
    return {next, std::clamp(raw, config.output_min, config.output_max)};
}

}  // namespace doogle::control
