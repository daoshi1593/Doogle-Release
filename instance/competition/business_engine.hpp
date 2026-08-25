#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "instance/api/business_contract.hpp"

namespace doogle::competition {

struct BusinessPhaseDefinition {
    std::string_view key;
    std::string_view description;
};

struct BusinessEngineConfig {
    std::array<int, 6> start_phase{1, 1, 1, 1, 1, 1};
    std::chrono::milliseconds nominal_tick{50};
    bool continue_after_soft_timeout{true};
    bool stage6_allow_partial_range{};
};

class BusinessEngine {
public:
    explicit BusinessEngine(BusinessEngineConfig config = {});

    [[nodiscard]] BusinessDecision tick(StageId stage, const SensorFrame& frame);
    void reset(StageId stage);
    [[nodiscard]] const BusinessEngineConfig& config() const { return config_; }

private:
    struct StageState {
        int phase{};  // zero based index into business_phases(stage)
        int step{};
        int stable_frames{};
        int lost_frames{};
        int phase_ticks{};
        int cycle{};
        int channel{};
        int attempt{};
        int pass_count{};
        int miss_count{};
        int continuation_step{};
        std::array<bool, 5> visited_targets{};
        std::array<bool, 4> completed_target_classes{};
        bool initialized{};
        bool pose_latched{};
        bool turn_target_latched{};
        bool target_is_fallback{};
        bool target_action_sent{};
        double origin_x{};
        double origin_y{};
        double origin_yaw_deg{};
        double segment_origin_x{};
        double segment_origin_y{};
        double segment_origin_yaw_deg{};
        double target_yaw_deg{};
        double last_x{};
        double last_y{};
        perception::VisualTarget current_target{perception::VisualTarget::None};
        std::deque<cv::Point2d> trajectory;
        std::chrono::steady_clock::time_point phase_started{};
    };

    [[nodiscard]] BusinessDecision stage1(const SensorFrame&, StageState&);
    [[nodiscard]] BusinessDecision stage2(const SensorFrame&, StageState&);
    [[nodiscard]] BusinessDecision stage3(const SensorFrame&, StageState&);
    [[nodiscard]] BusinessDecision stage4(const SensorFrame&, StageState&);
    [[nodiscard]] BusinessDecision stage5(const SensorFrame&, StageState&);
    [[nodiscard]] BusinessDecision stage6(const SensorFrame&, StageState&);

    void enter_phase(StageId stage, StageState& state, int phase,
                     std::chrono::steady_clock::time_point now);
    void next_phase(StageId stage, StageState& state,
                    std::chrono::steady_clock::time_point now);
    void enter_step(StageState& state, int step);
    [[nodiscard]] double phase_seconds(const StageState& state,
                                       const SensorFrame& frame) const;
    [[nodiscard]] double segment_progress(const StageState& state,
                                          const PoseSample& pose) const;
    void latch_segment_pose(StageState& state, const PoseSample& pose);

    [[nodiscard]] BusinessDecision decision(StageId stage, const StageState& state,
                                            protocol::RobotControlCommand command,
                                            std::string action, std::string reason,
                                            bool fail_closed = false) const;
    [[nodiscard]] BusinessDecision hold(StageId stage, const StageState& state,
                                        std::string reason) const;
    [[nodiscard]] BusinessDecision complete(StageId stage, const StageState& state,
                                            std::string reason) const;

    static protocol::RobotControlCommand motion(int mode, int gait, double vx = 0.0,
                                                 double vy = 0.0, double wz = 0.0,
                                                 double body_height = 0.235,
                                                 int duration_ms = 0);

    BusinessEngineConfig config_;
    std::array<StageState, 6> states_{};
    perception::DepthSlopeFollower depth_follower_{};
};

[[nodiscard]] std::optional<StageId> parse_stage_id(std::string_view value);
[[nodiscard]] std::string_view stage_name(StageId stage);
[[nodiscard]] const std::vector<BusinessPhaseDefinition>& business_phases(StageId stage);
[[nodiscard]] std::optional<int> parse_business_phase(StageId stage, std::string_view selector);
[[nodiscard]] const std::vector<std::string_view>& stage4_business_nodes();

}  // namespace doogle::competition
