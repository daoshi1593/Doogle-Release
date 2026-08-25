#pragma once

namespace doogle {

enum class MotionMode { Stand = 0, Walk = 11, Stop = 12 };

struct MotionIntent {
    MotionMode mode{MotionMode::Stand};
    int gait_id{0};
    int contact{0};
    float vx{0.0F};
    float vy{0.0F};
    float wz{0.0F};
    float body_height{0.0F};
};

}  // namespace doogle
