# Doogle

Doogle 是机器狗竞赛业务的原生 C++20 实现。仓库包含完整的 Service–Instance 架构、Stage1–6 业务状态机、感知算法、传感器协议、运动命令链路、离线 replay 和 software simulation tests，不依赖 Python 运行时。

当前版本完成 software-level business parity：业务 phase、动作顺序、触发条件、fallback、恢复入口和 fail-closed 契约均由 C++ 实现并通过合成传感器与闭环运动仿真测试。项目未声明具体机器人或场地上的硬件在环验证结果。

使用 Doogle infra 接入自有业务，请阅读 [INTEGRATION.md](INTEGRATION.md)。

## 架构

仓库只有两个源码目录：

- `service/`：无状态、可复用的算法与协议，包括 vision、Depth/ToF、PID、geometry、image/pose/sensor/audio/command codec 和 motion port。
- `instance/`：持有状态和资源的业务实例，包括 Stage1–6、phase orchestration、runner、relay、replay、sensor lifecycle 和 tests。

依赖方向保持单向：

```text
External Application
        |
        v
Instance Entry / BusinessEngine / Runtime
        |
        v
Service Perception / Control / Protocol / Port
```

External 只调用 Instance。Service 不依赖 Instance，也不保存比赛 phase、socket lifecycle 或线程状态。

## 数据流

```text
RGB / AI / Fisheye over ZMQ --------+
Pose / Depth / ToF over UDP --------+--> atomic SensorFrame
External detector observations -----+          |
                                                v
                                      Stage1–6 BusinessEngine
                                                |
                                                v
                                         BusinessDecision
                                     /          |          \
                              phase/action   speech key   full command
                                                           |
                                                           v
                                              relay heartbeat + stale stop
```

输入支持：

- BGR、RGB、BGRA、RGBA、Mono、YUYV、UYVY 和 JPEG image packet。
- `LC02` Pose、`DEP1` 分片 Depth 和双头 8×8 `TOF1`。
- OpenCV DNN ONNX 足球检测或内置圆形/黑白比例 heuristic。
- `BusinessObservations` 外部检测结果覆盖，用于接入使用者自己的 detector。
- `IBusinessInstance`、`ISensorProvider` 与 `RuntimeHost` 组合，用于在不复制 runner 的情况下接入自有业务。

输出是完整 `RobotControlCommand`，保留 mode、gait、velocity、RPY、body height、step height、duration 和 heartbeat `life_count`。

## 竞赛业务覆盖

| Stage | Business phases | 主要流程 |
| --- | ---: | --- |
| Stage1 | 7 | ready、Pose 投影石板路、81 步态、末段普通步态、右鱼眼切线、两次 ROI 横移、相对 90° 转向、左 ROI 离场 |
| Stage2 | 8 | 初始参考、双鱼眼严格居中、开场左移、前进四轮识别、完整 final exit、后退四轮、无红球侧移四轮、左黄 ROI 离场 |
| Stage3 | 4 | 起立、双鱼眼 ready、S 弯横向/航向闭环、丢线出弯、右黄线水平对齐 |
| Stage4 | 16 | 三通道、限高杆 pass-return、83/84 低姿步态、蓝块斜向绕行、虚线侧判定、目标 fallback、足球/可乐/橙球 p2 动作、原路返回、final lane |
| Stage5 | 10 | 身高调整、启动 ROI 直行、四段 Pose/lane 闭环、段间横移与转体、最终右转和 mode 16 jump、lie-down |
| Stage6 | 5 | sensor readiness、横移搜球、固定 1.6 m 接近、IMU/Pose PID 转向、RGB–Depth–ToF 推球、双条件终点、后退/转向/横移、ToF 精调与双次 kick |

Stage4 另外公开完整的 business-node catalog，覆盖 limit-height、blue-obstacle、target、yellow-line、relative-motion、speech 与 debug/resume contract。每个 Stage 都可按数字或 phase key 从指定业务节点恢复。

## 构建与测试

依赖：CMake 3.16+、C++20 compiler、OpenCV 4.5+、libzmq、zlib 和 pkg-config。

```bash
cmake --preset default
cmake --build --preset default -j
ctest --preset default
```

也可以显式选择构建目录：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Tests 使用合成图像、loopback UDP 和 Pose 动力学仿真，不连接真实设备。覆盖 phase catalog、遮挡球 mask 恢复、六阶段主流程、Stage4 fallback、fail-closed、完整 command relay、Depth 分片、ToF、image/audio codec 和 replay。

自定义业务 Runtime 另有原子测试覆盖 provider lifecycle、not-ready、end-of-stream、异常、输出失败、严格 tick budget、heartbeat 回绕、stop request 和 stop-on-exit。`doogle_custom_business_example` 是不依赖设备的完整接入示例。

## 自定义业务接口

新业务只需实现两个输入侧契约，并复用一个输出侧契约：

- `doogle::instance::IBusinessInstance`：接收原子 `SensorFrame`，返回 `BusinessDecision`。
- `doogle::instance::ISensorProvider`：负责 sensor lifecycle，并返回 ready、not-ready、end-of-stream 或 error。
- `doogle::ports::MotionSink`：发布完整 `RobotControlCommand`。

`doogle::runtime::RuntimeHost` 负责组合三者，统一处理 reset、tick budget、heartbeat、fail-closed、stop request、异常边界和退出 stop。已有 Stage1–6 可通过 `StageBusinessInstance` 适配到同一接口，新业务无需依赖具体 `BusinessEngine`。

需要直接复用本项目 file/ZMQ/UDP/ONNX 输入时，可使用 `StandardSensorProvider`；只有接入其他 middleware 时才需要自行实现 `ISensorProvider`。

完整代码见 `instance/example/custom_business.cpp` 和 [INTEGRATION.md](INTEGRATION.md)。

## 运行

单阶段 runner：

```bash
./build/doogle_runner \
  --stage stage4 \
  --start-phase obstacle-handling \
  --rgb-endpoint ipc:///tmp/doogle-rgb \
  --left-endpoint ipc:///tmp/doogle-left \
  --right-endpoint ipc:///tmp/doogle-right \
  --pose-port 17669 \
  --depth-port 17671 \
  --tof-port 17670 \
  --ticks 4000 \
  --period-ms 50 \
  --require-complete
```

`--start-phase` 接受 1-based 数字或 phase key，例如 `forward-recognition`、`channel-2-p0-test`、`finish-kick`。只有明确传入 `--motion-host` 与 `--motion-port` 时 runner 才会发送运动命令。

命令 relay：

```bash
./build/doogle_relay \
  --listen-host 127.0.0.1 --listen-port 17668 \
  --motion-host 127.0.0.1 --motion-port 17667 \
  --stale-ms 100
```

Relay 接收完整 command packet，按 5 ms heartbeat 重发；超过 stale timeout 自动替换为 stop，收到 SIGINT/SIGTERM 时再发送一次 stop。

Replay：

```bash
./build/doogle_replay business.trace
```

Replay 驱动同一个 `BusinessEngine`，输出包含 stage、phase、step、action、reason、command、fail-closed、degraded 和 speech 的 JSON lines。

## License

Doogle 使用 [MIT License](LICENSE)。
