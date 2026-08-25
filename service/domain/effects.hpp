#pragma once

#include <variant>

namespace doogle {

enum class AudioId { BallReached };
struct PlayAudio { AudioId id; };
struct EmitTrace { int event; };
using Effect = std::variant<PlayAudio, EmitTrace>;

}  // namespace doogle
