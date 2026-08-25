#pragma once

#include <atomic>

namespace doogle::runtime {

class Lifecycle {
public:
    void request_stop() noexcept { stop_requested_.store(true, std::memory_order_release); }
    [[nodiscard]] bool stop_requested() const noexcept {
        return stop_requested_.load(std::memory_order_acquire);
    }

private:
    std::atomic<bool> stop_requested_{false};
};

}  // namespace doogle::runtime
