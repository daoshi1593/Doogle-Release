#pragma once

#include <fstream>
#include <optional>
#include <ostream>
#include <string>

#include "instance/competition/business_engine.hpp"

namespace doogle::competition {

// Text trace schema (paths are std::quoted):
// stage timestamp_ms pose_valid x y yaw rgb ai left right depth
// football_valid football_x football_y football_radius football_score
class BusinessReplay {
public:
    explicit BusinessReplay(const std::string& path);
    [[nodiscard]] bool good() const { return input_.good(); }
    [[nodiscard]] std::optional<std::pair<StageId, SensorFrame>> next();

private:
    std::ifstream input_;
    std::uint32_t sequence_{};
};

void write_business_decision_json(std::ostream& output, const BusinessDecision& decision);

}  // namespace doogle::competition
