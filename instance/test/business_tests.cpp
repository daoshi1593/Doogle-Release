#include <algorithm>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>

#include <opencv2/imgproc.hpp>

#include "instance/competition/business_engine.hpp"
#include "instance/competition/business_replay.hpp"
#include "service/perception/geometry.hpp"

namespace {

using doogle::competition::BusinessDecision;
using doogle::competition::BusinessEngine;
using doogle::competition::BusinessEngineConfig;
using doogle::competition::PoseSample;
using doogle::competition::SensorFrame;
using doogle::competition::StageId;

cv::Mat black_frame(int width = 640, int height = 400) {
    return cv::Mat(height, width, CV_8UC3, cv::Scalar{0, 0, 0});
}

void integrate(PoseSample& pose, const BusinessDecision& decision, double seconds = 0.20) {
    const double yaw = pose.yaw_deg * CV_PI / 180.0;
    const double vx = decision.command.vel_des[0];
    const double vy = decision.command.vel_des[1];
    pose.x += (vx * std::cos(yaw) - vy * std::sin(yaw)) * seconds;
    pose.y += (vx * std::sin(yaw) + vy * std::cos(yaw)) * seconds;
    pose.yaw_deg = doogle::perception::normalize_angle_degrees(
        pose.yaw_deg + decision.command.vel_des[2] * seconds * 180.0 / CV_PI);
    ++pose.sequence;
}

BusinessEngine make_engine(StageId stage, int phase, bool partial_range = false) {
    BusinessEngineConfig config;
    config.start_phase[static_cast<std::size_t>(stage)] = phase;
    config.nominal_tick = std::chrono::milliseconds{100};
    config.stage6_allow_partial_range = partial_range;
    return BusinessEngine{config};
}

void test_phase_contract() {
    const std::array<std::size_t, 6> expected{7, 8, 4, 16, 10, 5};
    for (std::size_t index = 0; index < expected.size(); ++index) {
        const auto stage = static_cast<StageId>(index);
        const auto& phases = doogle::competition::business_phases(stage);
        assert(phases.size() == expected[index]);
        std::set<std::string_view> unique;
        for (const auto& phase : phases) assert(unique.insert(phase.key).second);
        assert(doogle::competition::parse_business_phase(stage, "1") == 1);
        assert(doogle::competition::parse_business_phase(stage, phases.back().key) ==
               static_cast<int>(phases.size()));
        assert(!doogle::competition::parse_business_phase(stage, "999"));
    }
    assert(doogle::competition::parse_stage_id("S4") == StageId::Stage4);
    assert(doogle::competition::parse_stage_id("stage_6") == StageId::Stage6);
    const auto& nodes = doogle::competition::stage4_business_nodes();
    assert(nodes.size() >= 50);
    assert(std::set<std::string_view>(nodes.begin(), nodes.end()).size() == nodes.size());
    assert(std::find(nodes.begin(), nodes.end(), "stage4.limit_height.return_yellow_center") !=
           nodes.end());
    assert(std::find(nodes.begin(), nodes.end(), "stage4.channels.three_channel_flow") !=
           nodes.end());
}

void test_ball_mask_occlusion_recovery() {
    cv::Mat half(240, 360, CV_8UC1, cv::Scalar{0});
    cv::circle(half, {180, 100}, 40, cv::Scalar{255}, -1);
    half.rowRange(0, 100).setTo(0);
    const auto recovered = doogle::perception::recover_ball_mask(half);
    assert(half.at<std::uint8_t>(75, 180) == 0);
    assert(recovered.at<std::uint8_t>(75, 180) == 255);

    cv::Mat noise(200, 200, CV_8UC1, cv::Scalar{0});
    cv::circle(noise, {100, 100}, 4, cv::Scalar{255}, -1);
    const auto unchanged = doogle::perception::recover_ball_mask(noise);
    assert(cv::countNonZero(unchanged) == cv::countNonZero(noise));
}

void test_business_replay_drives_full_engine_contract() {
    const auto path = std::filesystem::temp_directory_path() / "doogle_business_replay_test.trace";
    {
        std::ofstream output(path);
        output << "stage6 100 1 0 0 0 \"-\" \"-\" \"-\" \"-\" \"-\" "
                  "1 192 210 40 0.95\n";
    }
    doogle::competition::BusinessReplay replay{path.string()};
    auto item = replay.next();
    assert(item.has_value() && item->first == StageId::Stage6);
    assert(item->second.pose.has_value() && item->second.football.has_value());
    auto engine = make_engine(StageId::Stage6, 2, true);
    item->second.rgb = black_frame();
    const auto decision = engine.tick(item->first, item->second);
    std::ostringstream json;
    doogle::competition::write_business_decision_json(json, decision);
    assert(json.str().find("\"phase_key\":\"search_align_ball\"") != std::string::npos);
    assert(json.str().find("\"speech\":null") != std::string::npos);
    std::filesystem::remove(path);
}

void test_stage1_complete_sequence() {
    auto engine = make_engine(StageId::Stage1, 1);
    SensorFrame frame;
    frame.right_fisheye = black_frame();
    frame.left_fisheye = black_frame();
    frame.pose = PoseSample{};
    doogle::perception::YellowLineObservation line;
    line.detected = true;
    line.angle_deg = 0.0;
    frame.observations.right_yellow_line = line;
    frame.observations.right_bottom_yellow = doogle::perception::MarkerObservation{true, 1.0, 0.3};
    frame.observations.left_bottom_yellow = doogle::perception::MarkerObservation{true, 1.0, 0.1};
    bool user_gait{}, tail{}, turned{};
    for (int tick = 0; tick < 800; ++tick) {
        const auto result = engine.tick(StageId::Stage1, frame);
        user_gait |= result.action == "stone_road_user_gait";
        tail |= result.action == "stone_road_tail";
        turned |= result.action == "relative_yaw_turn";
        if (result.stage_complete) {
            assert(result.reason == "left_yellow_roi_filled" ||
                   result.reason == "max_forward_distance_reached");
            assert(user_gait && tail && turned);
            return;
        }
        integrate(*frame.pose, result);
    }
    assert(false && "Stage1 did not complete");
}

void test_stage2_four_cycle_and_final_exit() {
    auto engine = make_engine(StageId::Stage2, 4);
    SensorFrame frame;
    frame.pose = PoseSample{};
    frame.left_fisheye = black_frame(240, 200);
    frame.right_fisheye = black_frame(240, 200);
    cv::circle(frame.left_fisheye, {128, 90}, 32, {230, 157, 0}, -1);
    cv::circle(frame.right_fisheye, {128, 90}, 32, {230, 157, 0}, -1);
    int centered_cycles{};
    bool reached_final_exit{};
    std::string previous;
    for (int tick = 0; tick < 1200; ++tick) {
        const auto result = engine.tick(StageId::Stage2, frame);
        if (result.action == "center_ball_recognition" && previous != result.action) ++centered_cycles;
        if (result.phase_key == "final_exit") reached_final_exit = true;
        if (reached_final_exit && result.action == "final_exit_left_turn") {
            assert(centered_cycles >= 4);
            return;
        }
        previous = result.action;
        integrate(*frame.pose, result);
    }
    assert(false && "Stage2 forward recognition did not reach final exit");
}

void test_stage2_red_shift_contract() {
    auto engine = make_engine(StageId::Stage2, 4);
    SensorFrame frame;
    frame.pose = PoseSample{};
    frame.left_fisheye = black_frame(240, 200);
    frame.right_fisheye = black_frame(240, 200);
    cv::circle(frame.left_fisheye, {128, 90}, 32, {0, 0, 255}, -1);
    cv::circle(frame.right_fisheye, {128, 90}, 32, {0, 0, 255}, -1);
    bool shifted{}, returned{};
    for (int tick = 0; tick < 500; ++tick) {
        const auto result = engine.tick(StageId::Stage2, frame);
        shifted |= result.action == "red_ball_side_shift";
        returned |= result.action == "red_ball_return_shift";
        integrate(*frame.pose, result);
        if (shifted && returned) return;
    }
    assert(false && "Stage2 red shift and return were not both executed");
}

void test_stage3_closed_loop_and_exit_align() {
    auto engine = make_engine(StageId::Stage3, 3);
    SensorFrame frame;
    frame.left_fisheye = black_frame();
    frame.right_fisheye = black_frame();
    doogle::perception::YellowLineObservation line;
    line.detected = true;
    line.angle_deg = 8.0;
    frame.observations.left_yellow_line = line;
    frame.observations.right_yellow_line = line;
    auto result = engine.tick(StageId::Stage3, frame);
    assert(result.action == "s_curve_closed_loop" || result.action == "s_curve_hold_yaw");
    line.detected = false;
    frame.observations.left_yellow_line = line;
    frame.observations.right_yellow_line = line;
    for (int i = 0; i < 12; ++i) result = engine.tick(StageId::Stage3, frame);
    line.detected = true;
    line.angle_deg = 0.0;
    frame.observations.right_yellow_line = line;
    for (int i = 0; i < 20; ++i) {
        result = engine.tick(StageId::Stage3, frame);
        if (result.stage_complete) return;
    }
    assert(false && "Stage3 did not finish right horizontal alignment");
}

void test_stage4_limit_height_pass_target_return() {
    auto engine = make_engine(StageId::Stage4, 13);
    SensorFrame frame;
    frame.rgb = black_frame();
    frame.left_fisheye = black_frame();
    frame.right_fisheye = black_frame();
    frame.pose = PoseSample{};
    doogle::perception::LimitBarObservation limit;
    limit.detected = true;
    limit.near = true;
    limit.beam = {120, 180, 400, 30};
    frame.observations.limit_bar = limit;
    frame.observations.top_red = doogle::perception::MarkerObservation{true, 1.0, 0.5};
    doogle::perception::YellowLineObservation line;
    line.detected = true;
    line.angle_deg = 0.0;
    frame.observations.left_yellow_line = line;
    frame.football = doogle::perception::FootballObservation{
        true, {260, 170, 100, 100}, {310.0, 220.0}, 50.0, 0.95, 0.2, 0.5, "synthetic"};
    bool crawled{}, p2{}, target_action{}, returned{};
    for (int tick = 0; tick < 1800; ++tick) {
        const auto result = engine.tick(StageId::Stage4, frame);
        crawled |= result.action == "limit_height_low_crawl" && result.command.mode == 62 &&
                   result.command.gait_id == 83;
        p2 |= result.action == "limit_height_p0_to_p2";
        target_action |= result.action.starts_with("football_") ||
                         result.action == "p2_target_action_start";
        returned |= result.action == "limit_height_low_crawl_back" &&
                    result.command.gait_id == 84;
        if (result.action == "limit_height_channel_complete") {
            assert(crawled && p2 && target_action && returned);
            return;
        }
        integrate(*frame.pose, result);
    }
    assert(false && "Stage4 limit-height channel did not complete");
}

void test_stage4_target_fallback_contract() {
    auto engine = make_engine(StageId::Stage4, 13);
    SensorFrame frame;
    frame.rgb = black_frame();
    frame.left_fisheye = black_frame();
    frame.right_fisheye = black_frame();
    frame.pose = PoseSample{};
    doogle::perception::LimitBarObservation limit;
    limit.detected = true;
    limit.near = true;
    limit.beam = {120, 180, 400, 30};
    frame.observations.limit_bar = limit;
    frame.observations.top_red = doogle::perception::MarkerObservation{true, 1.0, 0.5};
    doogle::perception::YellowLineObservation line;
    line.detected = true;
    frame.observations.left_yellow_line = line;
    for (int tick = 0; tick < 1000; ++tick) {
        const auto result = engine.tick(StageId::Stage4, frame);
        if (result.action == "p2_target_action_start") {
            assert(result.target == doogle::perception::VisualTarget::Football);
            return;
        }
        integrate(*frame.pose, result);
    }
    assert(false && "Stage4 target fallback did not select remaining target");
}

void test_stage5_closed_loop_and_final_jump() {
    auto engine = make_engine(StageId::Stage5, 3);
    SensorFrame frame;
    frame.pose = PoseSample{};
    frame.depth = cv::Mat(480, 640, CV_16UC1, cv::Scalar{2000});
    cv::rectangle(frame.depth, {180, 0, 280, 480}, cv::Scalar{1000}, -1);
    frame.right_fisheye = black_frame();
    bool reached_tail{};
    for (int tick = 0; tick < 600; ++tick) {
        const auto result = engine.tick(StageId::Stage5, frame);
        if (result.phase_key == "first_post_walk_tail") {
            reached_tail = true;
            break;
        }
        integrate(*frame.pose, result);
    }
    assert(reached_tail);

    auto final_engine = make_engine(StageId::Stage5, 10);
    frame.pose = PoseSample{};
    bool jumped{};
    for (int tick = 0; tick < 500; ++tick) {
        const auto result = final_engine.tick(StageId::Stage5, frame);
        jumped |= result.action == "final_forward_jump" && result.command.mode == 16 &&
                  result.command.gait_id == 1;
        if (result.stage_complete) {
            assert(jumped);
            assert(result.command.mode == 7);
            return;
        }
        integrate(*frame.pose, result);
    }
    assert(false && "Stage5 final jump did not complete");
}

doogle::perception::TofFrame ready_tof() {
    doogle::perception::TofFrame tof;
    tof.left.fill(20.0F);
    tof.right.fill(20.0F);
    for (int row = 4; row < 8; ++row) {
        for (int col = 0; col < 4; ++col) {
            tof.left[row * 8 + col] = 15.93F;
            tof.right[(7 - row) * 8 + col] = 15.0F;
        }
    }
    return tof;
}

void test_stage6_full_business_sequence() {
    auto engine = make_engine(StageId::Stage6, 2);
    SensorFrame frame;
    frame.rgb = black_frame();
    frame.depth = cv::Mat(480, 640, CV_16UC1, cv::Scalar{500});
    frame.pose = PoseSample{};
    frame.football = doogle::perception::FootballObservation{
        true, {150, 170, 80, 80}, {192.0, 210.0}, 40.0, 0.95, 0.2, 0.5, "synthetic"};
    frame.tof = ready_tof();
    bool approached{}, pushed{}, kicked{};
    for (int tick = 0; tick < 2200; ++tick) {
        if (pushed) frame.observations.yellow_finish_boundary = true;
        const auto result = engine.tick(StageId::Stage6, frame);
        approached |= result.action == "fixed_approach_after_ball";
        pushed |= result.action == "football_push_transition" ||
                  result.action == "football_push_tof";
        kicked |= result.action == "tof_kick_forward";
        if (result.stage_complete) {
            assert(approached && pushed && kicked);
            return;
        }
        integrate(*frame.pose, result);
    }
    assert(false && "Stage6 full sequence did not complete");
}

void test_fail_closed_contracts() {
    SensorFrame frame;
    auto stage4 = make_engine(StageId::Stage4, 4);
    auto result = stage4.tick(StageId::Stage4, frame);
    assert(result.fail_closed && result.command.mode == 12);
    auto stage6 = make_engine(StageId::Stage6, 1);
    frame.rgb = black_frame();
    frame.pose = PoseSample{};
    result = stage6.tick(StageId::Stage6, frame);
    assert(result.fail_closed && result.reason == "depth_and_tof_not_ready");
    frame.control_safe = false;
    result = stage6.tick(StageId::Stage6, frame);
    assert(result.fail_closed && result.reason == "control_status_unsafe");
}

}  // namespace

int main() {
    test_phase_contract();
    test_ball_mask_occlusion_recovery();
    test_business_replay_drives_full_engine_contract();
    test_stage1_complete_sequence();
    test_stage2_four_cycle_and_final_exit();
    test_stage2_red_shift_contract();
    test_stage3_closed_loop_and_exit_align();
    test_stage4_limit_height_pass_target_return();
    test_stage4_target_fallback_contract();
    test_stage5_closed_loop_and_final_jump();
    test_stage6_full_business_sequence();
    test_fail_closed_contracts();
    return 0;
}
