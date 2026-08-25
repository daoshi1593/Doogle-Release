#include "instance/runtime/motion_relay.hpp"

#include "service/control/motion.hpp"

namespace doogle::runtime {

MotionRelay::MotionRelay(CommandSlot& slot, ports::MotionSink& sink,
                         std::chrono::milliseconds stale_after)
    : slot_(slot), sink_(sink), heartbeat_(slot), stale_after_(stale_after) {}

void MotionRelay::mark_command_update(std::chrono::steady_clock::time_point now) {
    last_update_ = now;
}

void MotionRelay::tick(std::chrono::steady_clock::time_point now) {
    if (last_update_ == std::chrono::steady_clock::time_point{} || now - last_update_ > stale_after_) {
        slot_.store(control::stop_motion());
    }
    sink_.publish(heartbeat_.tick());
}

}  // namespace doogle::runtime
