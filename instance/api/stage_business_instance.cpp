#include "instance/api/stage_business_instance.hpp"

namespace doogle::instance {

StageBusinessInstance::StageBusinessInstance(
    competition::StageId stage,
    competition::BusinessEngineConfig config)
    : stage_(stage), engine_(config) {}

std::string_view StageBusinessInstance::name() const noexcept {
    return competition::stage_name(stage_);
}

competition::BusinessDecision StageBusinessInstance::tick(
    const competition::SensorFrame& frame) {
    return engine_.tick(stage_, frame);
}

void StageBusinessInstance::reset() {
    engine_.reset(stage_);
}

}  // namespace doogle::instance
