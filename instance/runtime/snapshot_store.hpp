#pragma once

#include <mutex>
#include <optional>

namespace doogle::runtime {

template <typename T>
class SnapshotStore {
public:
    void publish(T value) {
        std::lock_guard lock(mutex_);
        latest_ = std::move(value);
    }

    [[nodiscard]] std::optional<T> latest() const {
        std::lock_guard lock(mutex_);
        return latest_;
    }

private:
    mutable std::mutex mutex_;
    std::optional<T> latest_;
};

}  // namespace doogle::runtime
