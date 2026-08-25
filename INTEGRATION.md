# 接入 Doogle Infra

本文说明如何把自有 detector、sensor transport、业务状态机或 motion sink 接入 Doogle。所有接入都遵循 Service–Instance Driven Architecture（SIDA）：External 只调用 Instance，Instance 编排 Service，Service 不依赖 Instance。

## 1. 选择接口

| 需求 | 接口 | CMake target |
| --- | --- | --- |
| 运行 Stage1–6 | `doogle::stage6::run_stage` | `doogle_instance` |
| 将自己的传感器数据输入现有业务 | `doogle::competition::BusinessEngine` | `doogle_instance_competition_core` |
| 使用自己的 detector | `SensorFrame::observations` / `SensorFrame::football` | `doogle_instance_competition_core` |
| 编写自己的业务 Instance | `service/perception`、`service/control`、`service/protocol` | 对应 `doogle_service_*` |
| 新增 transport | Service codec + Instance runtime owner | `doogle_instance_runtime` |
| 新增运动输出 | 实现 `doogle::ports::MotionSink` | `doogle_service_domain` |

将项目作为子目录引入：

```cmake
add_subdirectory(path/to/Doogle)

add_executable(my_business main.cpp)
target_link_libraries(my_business PRIVATE doogle_instance)
```

只复用算法与协议：

```cmake
target_link_libraries(my_business PRIVATE
    doogle_service_perception
    doogle_service_control
    doogle_service_protocol
)
```

## 2. 配置现有 Instance

`StageRunConfig` 组合 file、ZMQ、UDP 和 model 输入，然后驱动 `BusinessEngine`：

```cpp
#include "instance/competition/stage_app.hpp"

int main() {
    doogle::stage6::StageRunConfig config;
    config.stage_name = "stage6";
    config.start_phase = "search-align-ball";
    config.rgb_endpoint = "ipc:///tmp/doogle-rgb";
    config.pose_port = 17669;
    config.depth_port = 17671;
    config.tof_port = 17670;
    config.ticks = 4000;
    config.period = std::chrono::milliseconds{50};
    config.allow_incomplete = false;
    return doogle::stage6::run_stage(config);
}
```

返回码：`0` 表示成功或允许 incomplete；`2` 表示参数错误；`3` 表示输入、socket 或 model 打开失败；`4` 表示启用 require-complete 后 tick budget 耗尽。

## 3. 直接驱动 BusinessEngine

```cpp
#include "instance/competition/business_engine.hpp"

doogle::competition::BusinessEngineConfig config;
config.start_phase[static_cast<std::size_t>(
    doogle::competition::StageId::Stage4
)] = 9;
config.nominal_tick = std::chrono::milliseconds{50};

doogle::competition::BusinessEngine engine{config};
doogle::competition::SensorFrame frame;
frame.rgb = current_rgb;
frame.depth = current_depth;
frame.pose = doogle::competition::PoseSample{
    .x = pose_x,
    .y = pose_y,
    .yaw_deg = pose_yaw_deg,
    .source_timestamp = source_timestamp,
    .sequence = sequence,
};
frame.control_safe = control_is_safe;
frame.now = std::chrono::steady_clock::now();

const auto decision = engine.tick(
    doogle::competition::StageId::Stage4,
    frame
);

if (decision.fail_closed) {
    publish_stop();
} else {
    publish(decision.command);
}

if (decision.speech) {
    announce(*decision.speech);
}
```

`BusinessDecision` 的稳定字段：

- `phase` / `phase_key` / `step`：业务位置。
- `command`：完整 `RobotControlCommand`。
- `target` / `action` / `reason`：本 tick 的决策依据。
- `speech`：可选的语音 semantic key，由使用者自己的 audio infra 消费。
- `stage_complete`：阶段完成。
- `fail_closed`：缺失关键 sensor、unsafe control 或 stop request。
- `degraded`：允许 soft timeout 后继续时留下的显式标记。

## 4. 接入自己的 Detector

使用者可以让 Doogle 从图像执行内置检测，也可以写入 observation override：

```cpp
doogle::perception::ColorDetection detection;
detection.target = doogle::perception::VisualTarget::BlueCube;
detection.component.detected = true;
detection.component.center_x_ratio = detector_center_x;
detection.component.area_ratio = detector_area_ratio;
detection.confidence = detector_confidence;

frame.observations.blue_cube = detection;
frame.football = my_football_observation;
frame.observations.left_yellow_line = my_left_line;
frame.observations.right_yellow_line = my_right_line;
frame.observations.yellow_finish_boundary = my_finish_gate;
```

Override 缺失时，BusinessEngine 自动调用 native C++ perception。Override 的坐标约定和内置算法一致：图像归一化中心范围为 `[0, 1]`，Pose yaw 使用 degree，Depth 使用 `CV_16UC1` millimeter 或 floating-point meter。

## 5. 编写自己的业务 Instance

业务状态必须放在 `instance/`；Service 只提供无状态算法。External 应只构造并调用一个 Instance：

```cpp
#include "service/perception/vision.hpp"
#include "service/protocol/robot_control_cmd.hpp"

class MyBusinessInstance {
public:
    doogle::protocol::RobotControlCommand tick(const cv::Mat& frame) {
        const auto target = doogle::perception::detect_colored_target(
            frame,
            doogle::perception::VisualTarget::BlueBall
        );
        doogle::protocol::RobotControlCommand command;
        command.contact = 15;
        if (!target.component.detected) {
            command.mode = 12;
            return command;
        }
        command.mode = 11;
        command.gait_id = 27;
        const double error = target.component.center_x_ratio - 0.5;
        command.vel_des = {
            0.08F,
            static_cast<float>(std::clamp(-0.35 * error, -0.08, 0.08)),
            0.0F,
        };
        return command;
    }
};
```

## 6. 接入新的 Sensor Provider

拆分为两层：

1. 在 `service/protocol` 实现纯 codec，只负责 wire bytes 与领域数据之间的转换。
2. 在 `instance/runtime` 持有 socket、thread、timeout、reassembly 和 latest snapshot。

参考实现：

- `service/protocol/image_packet.*`：RAW/JPEG image codec。
- `service/protocol/pose_protocol.*`：LC02 Pose codec。
- `service/protocol/sensor_packets.*`：DEP1/TOF1 codec。
- `instance/runtime/zmq_frame_receiver.*`：ZMQ lifecycle。
- `instance/runtime/udp_sensor_hub.*`：Pose/Depth/ToF UDP owner。
- `instance/runtime/depth_assembler.*`：Depth fragment reassembly。

Provider 必须满足：不发布 partial frame；sequence 和 timestamp 可检查；Pose 有限且原子；Depth 收齐后才提交；错误或过期数据返回 not-ready，让业务 fail-closed。

## 7. 接入 Motion Sink 与 Relay

自有输出实现 `doogle::ports::MotionSink`：

```cpp
#include "service/port/motion_sink.hpp"

class MyMotionSink final : public doogle::ports::MotionSink {
public:
    void publish(const doogle::protocol::RobotControlCommand& command) override {
        transport_.send(command);
    }
};
```

长期运行建议把 command packet 发到 `doogle_relay`。Relay 的接收端写入 `CommandSlot`，heartbeat 只更新 `life_count`，不会丢失 RPY、body height、step height 或 duration；stale timeout 和进程退出都会发送 stop。

## 8. Replay Contract

每行 trace schema：

```text
stage timestamp_ms pose_valid x y yaw "rgb" "ai" "left" "right" "depth" football_valid football_x football_y football_radius football_score
```

未提供的 image path 写 `"-"`。同一 trace 可包含不同 Stage，replay 为每个 Stage 保留独立状态。

## 9. 验收清单

- [ ] External 只调用 Instance。
- [ ] 业务状态只存在于 Instance。
- [ ] Service 无 socket、thread 和业务 phase。
- [ ] Sensor snapshot 完整、原子、可判定新鲜度。
- [ ] 缺少核心输入、unsafe control 和 stop request 均 fail-closed。
- [ ] Motion endpoint 必须显式启用。
- [ ] Relay 保留完整 command 并启用 heartbeat/stale stop。
- [ ] 新业务包含正常、fallback、timeout、resume 和 failure tests。
- [ ] 配置与 trace 不包含真实设备地址、凭据、个人路径或私有模型。
