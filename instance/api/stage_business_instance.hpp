#pragma once

#include "instance/api/business_instance.hpp"
#include "instance/competition/business_engine.hpp"

namespace doogle::instance {

class StageBusinessInstance final : public IBusinessInstance {
public:
    explicit StageBusinessInstance(
        competition::StageId stage,
        competition::BusinessEngineConfig config = {});

    [[nodiscard]] std::string_view name() const noexcept override;
    [[nodiscard]] competition::BusinessDecision tick(
        const competition::SensorFrame& frame) override;
    void reset() override;

    [[nodiscard]] competition::StageId stage() const noexcept { return stage_; }
    [[nodiscard]] const competition::BusinessEngine& engine() const noexcept { return engine_; }

private:
    competition::StageId stage_;
    competition::BusinessEngine engine_;
};

}  // namespace doogle::instance
