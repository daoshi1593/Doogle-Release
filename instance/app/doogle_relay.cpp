#include <chrono>
#include <csignal>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include "instance/runtime/command_slot.hpp"
#include "instance/runtime/lifecycle.hpp"
#include "instance/runtime/motion_relay.hpp"
#include "instance/runtime/udp_command_receiver.hpp"
#include "service/adapter/udp_motion_sink.hpp"

namespace {

doogle::runtime::Lifecycle* active_lifecycle = nullptr;

void handle_signal(int) {
    if (active_lifecycle != nullptr) active_lifecycle->request_stop();
}

bool unsigned_value(const char* text, unsigned long maximum, unsigned long& output) {
    try {
        std::size_t end{};
        const auto parsed = std::stoul(text, &end);
        if (text[end] != '\0' || parsed > maximum) return false;
        output = parsed;
        return true;
    } catch (...) {
        return false;
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string listen_host{"127.0.0.1"};
    std::uint16_t listen_port{17668};
    std::string motion_host{"127.0.0.1"};
    std::uint16_t motion_port{17667};
    std::chrono::milliseconds stale_after{100};
    for (int index = 1; index < argc; ++index) {
        const std::string key{argv[index]};
        const auto value = [&]() -> const char* {
            return index + 1 < argc ? argv[++index] : nullptr;
        };
        if (key == "--listen-host") {
            const auto* item = value();
            if (!item) return 2;
            listen_host = item;
        } else if (key == "--motion-host") {
            const auto* item = value();
            if (!item) return 2;
            motion_host = item;
        } else if (key == "--listen-port" || key == "--motion-port" || key == "--stale-ms") {
            const auto* item = value();
            unsigned long parsed{};
            const unsigned long maximum = key == "--stale-ms"
                                               ? static_cast<unsigned long>(std::numeric_limits<int>::max())
                                               : 65535UL;
            if (!item || !unsigned_value(item, maximum, parsed) || parsed == 0) return 2;
            if (key == "--listen-port") listen_port = static_cast<std::uint16_t>(parsed);
            else if (key == "--motion-port") motion_port = static_cast<std::uint16_t>(parsed);
            else stale_after = std::chrono::milliseconds{parsed};
        } else {
            std::cerr << "unknown_option=" << key << '\n';
            return 2;
        }
    }

    doogle::runtime::Lifecycle lifecycle;
    active_lifecycle = &lifecycle;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    doogle::runtime::UdpCommandReceiver receiver{listen_host, listen_port};
    if (!receiver.open()) {
        std::cerr << "command_receiver_open_failed\n";
        return 3;
    }
    doogle::runtime::CommandSlot slot;
    doogle::adapters::UdpMotionSink sink{motion_host, motion_port};
    doogle::runtime::MotionRelay relay{slot, sink, stale_after};
    while (!lifecycle.stop_requested()) {
        const auto now = std::chrono::steady_clock::now();
        if (const auto command = receiver.poll()) {
            slot.store(*command);
            relay.mark_command_update(now);
        }
        relay.tick(now);
        std::this_thread::sleep_for(doogle::runtime::kHeartbeatPeriod);
    }
    doogle::protocol::RobotControlCommand stop;
    stop.mode = 12;
    stop.contact = 15;
    slot.store(stop);
    relay.mark_command_update(std::chrono::steady_clock::now());
    relay.tick(std::chrono::steady_clock::now());
    return 0;
}
