#include "service/control/motion.hpp"

namespace doogle::control {
MotionIntent stop_motion() { return MotionIntent{MotionMode::Stop, 0, 0, 0.0F, 0.0F, 0.0F, 0.0F}; }
}  // namespace doogle::control
