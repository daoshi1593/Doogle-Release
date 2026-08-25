#include <cassert>
#include <cmath>
#include "service/protocol/lcm/command_codec.hpp"
#include "service/control/pid.hpp"
#include "service/perception/geometry.hpp"
#include "instance/competition/stages.hpp"
#include "instance/competition/stage_app.hpp"
#include "instance/runtime/motion_relay.hpp"
#include "service/protocol/pose_protocol.hpp"
#include "service/protocol/sensor_packets.hpp"
#include "instance/runtime/heartbeat.hpp"
#include "instance/competition/reducer.hpp"

int main() {
    using namespace doogle;
    stage6::Config config;
    ControlState state;
    Observation visible;
    visible.ball = BallObservation{0.4F, 0.02F, 1.0F};
    const auto approach = stage6::reduce(config, state, visible);
    assert(approach.next_state.stage == Stage::Approach);
    assert(std::fabs(approach.desired_motion.vy - config.approach_vy_limit) < 1e-6F);

    visible.ball->distance = 0.1F;
    const auto kick = stage6::reduce(config, approach.next_state, visible);
    assert(kick.next_state.stage == Stage::Kick);
    assert(kick.effects.size() == 1);

    runtime::CommandSlot slot;
    slot.store(kick.desired_motion);
    runtime::Heartbeat heartbeat(slot);
    auto command = heartbeat.tick();
    assert(command.life_count == 1);
    for (int i = 0; i < 128; ++i) command = heartbeat.tick();
    assert(command.life_count == 1);

    command.value = 42;
    command.duration = 7;
    command.vel_des = {1.0F, -2.0F, 3.0F};
    const auto packet = protocol::encode_command(command);
    assert(packet.size() == 124);
    const auto decoded = protocol::decode_command(packet);
    assert(decoded.value == 42 && decoded.duration == 7);
    assert(decoded.vel_des == command.vel_des);

    const auto pid = control::update_pid({2.0, 0.0, 0.0, -1.0, 1.0}, {}, 1.0, 0.1);
    assert(pid.output == 1.0);
    assert(perception::normalize_angle_degrees(270.0) == -90.0);
    const auto geometry = perception::camera_ball_to_body({388.666, 388.178, 325.118, 233.308},
                                                          325.118, 233.308, 20.0, {1.0}, {0.2}, {0.0});
    assert(geometry.has_value());
    assert(perception::tof_distance({20.0}, {10.0}, {30.0}).has_value());
    auto stages = stage6::advance({}, true);
    assert(stages.stage == stage6::StageId::Stage2);

    struct Sink final : ports::MotionSink {
        protocol::RobotControlCommand command{};
        void publish(const protocol::RobotControlCommand& value) override { command = value; }
    } sink;
    runtime::CommandSlot relay_slot;
    runtime::MotionRelay relay{relay_slot, sink, std::chrono::milliseconds{100}};
    const auto now = std::chrono::steady_clock::now();
    relay.mark_command_update(now);
    relay.tick(now);
    assert(sink.command.life_count == 1);
    assert(stage6::run_stage("stage1") == 0);
    assert(stage6::run_stage("invalid") == 2);
    assert(!protocol::parse_lc02_frame({0, 1, 2}, "global_to_robot"));
    assert(!protocol::decode_pose_payload({}));
    assert(!protocol::valid_tof_packet({}));
    return 0;
}
