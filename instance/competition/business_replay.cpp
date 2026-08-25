#include "instance/competition/business_replay.hpp"

#include <iomanip>
#include <limits>

#include <opencv2/imgcodecs.hpp>

namespace doogle::competition {
namespace {

cv::Mat load_image(const std::string& path, int flags) {
    return path.empty() || path == "-" ? cv::Mat{} : cv::imread(path, flags);
}

}  // namespace

BusinessReplay::BusinessReplay(const std::string& path) : input_(path) {}

std::optional<std::pair<StageId, SensorFrame>> BusinessReplay::next() {
    std::string stage_text;
    long long timestamp_ms{};
    int pose_valid{};
    double x{}, y{}, yaw{};
    std::string rgb, ai, left, right, depth;
    int football_valid{};
    double football_x{}, football_y{}, football_radius{}, football_score{};
    if (!(input_ >> stage_text >> timestamp_ms >> pose_valid >> x >> y >> yaw >> std::quoted(rgb) >>
          std::quoted(ai) >> std::quoted(left) >> std::quoted(right) >> std::quoted(depth) >>
          football_valid >> football_x >> football_y >> football_radius >> football_score))
        return std::nullopt;
    const auto stage = parse_stage_id(stage_text);
    if (!stage) return std::nullopt;
    SensorFrame frame;
    frame.now = std::chrono::steady_clock::time_point{std::chrono::milliseconds{timestamp_ms}};
    if (pose_valid != 0)
        frame.pose = PoseSample{x, y, yaw, static_cast<std::uint64_t>(timestamp_ms), ++sequence_};
    frame.rgb = load_image(rgb, cv::IMREAD_COLOR);
    frame.ai = load_image(ai, cv::IMREAD_COLOR);
    frame.left_fisheye = load_image(left, cv::IMREAD_COLOR);
    frame.right_fisheye = load_image(right, cv::IMREAD_COLOR);
    frame.depth = load_image(depth, cv::IMREAD_ANYDEPTH);
    if (football_valid != 0) {
        frame.football = perception::FootballObservation{
            true,
            {static_cast<int>(football_x - football_radius),
             static_cast<int>(football_y - football_radius),
             static_cast<int>(football_radius * 2.0),
             static_cast<int>(football_radius * 2.0)},
            {football_x, football_y},
            football_radius,
            football_score,
            0.0,
            0.0,
            "replay_trace"};
    }
    return std::pair{*stage, std::move(frame)};
}

void write_business_decision_json(std::ostream& output, const BusinessDecision& decision) {
    output << std::setprecision(9) << "{\"stage\":\"" << stage_name(decision.stage)
           << "\",\"phase\":" << decision.phase << ",\"step\":" << decision.step
           << ",\"phase_key\":\"" << decision.phase_key << "\",\"action\":\""
           << decision.action << "\",\"reason\":\"" << decision.reason
           << "\",\"mode\":" << static_cast<int>(decision.command.mode)
           << ",\"gait_id\":" << static_cast<int>(decision.command.gait_id)
           << ",\"vx\":" << decision.command.vel_des[0]
           << ",\"vy\":" << decision.command.vel_des[1]
           << ",\"wz\":" << decision.command.vel_des[2]
           << ",\"complete\":" << (decision.stage_complete ? "true" : "false")
           << ",\"fail_closed\":" << (decision.fail_closed ? "true" : "false")
           << ",\"degraded\":" << (decision.degraded ? "true" : "false")
           << ",\"speech\":";
    if (decision.speech) output << '\"' << *decision.speech << '\"';
    else output << "null";
    output << '}';
}

}  // namespace doogle::competition
