#pragma once

#include <chrono>

namespace doogle::ports {

class Clock {
public:
    virtual ~Clock() = default;
    [[nodiscard]] virtual std::chrono::steady_clock::time_point now() const = 0;
};

class SteadyClock final : public Clock {
public:
    [[nodiscard]] std::chrono::steady_clock::time_point now() const override {
        return std::chrono::steady_clock::now();
    }
};

}  // namespace doogle::ports
