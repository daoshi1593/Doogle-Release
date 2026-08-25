#pragma once

#include <chrono>

#include "service/port/motion_sink.hpp"
#include "instance/runtime/command_slot.hpp"
#include "instance/runtime/heartbeat.hpp"

namespace doogle::runtime {

class MotionRelay {
public:
    MotionRelay(CommandSlot& slot, ports::MotionSink& sink,
                std::chrono::milliseconds stale_after = std::chrono::milliseconds{100});
    void tick(std::chrono::steady_clock::time_point now);
    void mark_command_update(std::chrono::steady_clock::time_point now);

private:
    CommandSlot& slot_;
    ports::MotionSink& sink_;
    Heartbeat heartbeat_;
    std::chrono::milliseconds stale_after_;
    std::chrono::steady_clock::time_point last_update_{};
};

}  // namespace doogle::runtime
