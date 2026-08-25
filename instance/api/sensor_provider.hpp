#pragma once

#include <chrono>
#include <string>
#include <utility>

#include "instance/api/business_contract.hpp"

namespace doogle::instance {

enum class SensorReadStatus {
    Ready,
    NotReady,
    EndOfStream,
    Error,
};

struct SensorRead {
    SensorReadStatus status{SensorReadStatus::NotReady};
    competition::SensorFrame frame{};
    std::string detail;

    [[nodiscard]] static SensorRead ready(competition::SensorFrame value) {
        return {SensorReadStatus::Ready, std::move(value), {}};
    }

    [[nodiscard]] static SensorRead not_ready(std::string reason = "sensor_not_ready") {
        return {SensorReadStatus::NotReady, {}, std::move(reason)};
    }

    [[nodiscard]] static SensorRead end_of_stream(std::string reason = "sensor_end_of_stream") {
        return {SensorReadStatus::EndOfStream, {}, std::move(reason)};
    }

    [[nodiscard]] static SensorRead error(std::string reason = "sensor_error") {
        return {SensorReadStatus::Error, {}, std::move(reason)};
    }
};

class ISensorProvider {
public:
    virtual ~ISensorProvider() = default;

    [[nodiscard]] virtual bool open() = 0;
    virtual void close() noexcept = 0;
    [[nodiscard]] virtual SensorRead read(std::chrono::milliseconds timeout) = 0;
};

}  // namespace doogle::instance
