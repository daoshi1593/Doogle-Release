#include "instance/runtime/standard_sensor_provider.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <utility>

#include <opencv2/dnn.hpp>
#include <opencv2/imgcodecs.hpp>

#include "instance/runtime/udp_sensor_hub.hpp"
#include "instance/runtime/zmq_frame_receiver.hpp"
#include "service/perception/vision.hpp"

namespace doogle::runtime {
namespace {

struct FrameSource {
    cv::Mat latest;
    std::unique_ptr<ZmqFrameReceiver> receiver;
};

bool configure_source(FrameSource& source, const std::string& path,
                      const std::string& endpoint, int image_flags) {
    if (!path.empty()) {
        source.latest = cv::imread(path, image_flags);
        if (source.latest.empty()) return false;
    }
    if (!endpoint.empty()) {
        source.receiver = std::make_unique<ZmqFrameReceiver>(endpoint);
        if (!source.receiver->open()) return false;
    }
    return true;
}

void refresh(FrameSource& source, std::chrono::milliseconds timeout) {
    if (!source.receiver) return;
    if (auto frame = source.receiver->receive(timeout)) source.latest = std::move(frame->frame);
}

bool contains_any_input(const competition::SensorFrame& frame) {
    return !frame.rgb.empty() || !frame.ai.empty() || !frame.left_fisheye.empty() ||
           !frame.right_fisheye.empty() || !frame.depth.empty() || frame.tof.has_value() ||
           frame.football.has_value() || frame.pose.has_value();
}

}  // namespace

struct StandardSensorProvider::Impl {
    FrameSource rgb;
    FrameSource ai;
    FrameSource left;
    FrameSource right;
    FrameSource depth;
    cv::dnn::Net football_network;
    std::unique_ptr<UdpSensorHub> sensor_hub;
    bool open{};
};

StandardSensorProvider::StandardSensorProvider(StandardSensorProviderConfig config)
    : config_(std::move(config)), impl_(std::make_unique<Impl>()) {}

StandardSensorProvider::~StandardSensorProvider() {
    close();
}

StandardSensorProvider::StandardSensorProvider(StandardSensorProvider&&) noexcept = default;
StandardSensorProvider& StandardSensorProvider::operator=(StandardSensorProvider&&) noexcept = default;

bool StandardSensorProvider::open() {
    close();
    impl_ = std::make_unique<Impl>();
    if (!configure_source(impl_->rgb, config_.rgb_path, config_.rgb_endpoint, cv::IMREAD_COLOR) ||
        !configure_source(impl_->ai, config_.ai_path, config_.ai_endpoint, cv::IMREAD_COLOR) ||
        !configure_source(impl_->left, config_.left_fisheye_path,
                          config_.left_fisheye_endpoint, cv::IMREAD_COLOR) ||
        !configure_source(impl_->right, config_.right_fisheye_path,
                          config_.right_fisheye_endpoint, cv::IMREAD_COLOR) ||
        !configure_source(impl_->depth, config_.depth_path,
                          config_.depth_endpoint, cv::IMREAD_ANYDEPTH)) {
        close();
        return false;
    }
    if (!config_.football_model_path.empty()) {
        try {
            impl_->football_network = cv::dnn::readNetFromONNX(config_.football_model_path);
        } catch (const cv::Exception&) {
            close();
            return false;
        }
    }

    UdpSensorConfig sensor_config;
    sensor_config.bind_host = config_.sensor_bind_host;
    sensor_config.pose_port = config_.pose_port;
    sensor_config.depth_port = config_.depth_port;
    sensor_config.tof_port = config_.tof_port;
    impl_->sensor_hub = std::make_unique<UdpSensorHub>(std::move(sensor_config));
    if (!impl_->sensor_hub->open()) {
        close();
        return false;
    }
    impl_->open = true;
    return true;
}

void StandardSensorProvider::close() noexcept {
    if (!impl_) return;
    if (impl_->sensor_hub) impl_->sensor_hub->close();
    for (FrameSource* source : std::array<FrameSource*, 5>{
             &impl_->rgb, &impl_->ai, &impl_->left, &impl_->right, &impl_->depth}) {
        if (source->receiver) source->receiver->close();
    }
    impl_->open = false;
}

instance::SensorRead StandardSensorProvider::read(std::chrono::milliseconds timeout) {
    if (!impl_ || !impl_->open) {
        return instance::SensorRead::error("standard_sensor_provider_not_open");
    }
    refresh(impl_->rgb, timeout);
    refresh(impl_->ai, std::chrono::milliseconds{0});
    refresh(impl_->left, std::chrono::milliseconds{0});
    refresh(impl_->right, std::chrono::milliseconds{0});
    refresh(impl_->depth, std::chrono::milliseconds{0});
    impl_->sensor_hub->poll();

    competition::SensorFrame frame;
    frame.rgb = impl_->rgb.latest;
    frame.ai = impl_->ai.latest;
    frame.left_fisheye = impl_->left.latest;
    frame.right_fisheye = impl_->right.latest;
    frame.depth = impl_->depth.latest;
    if (impl_->sensor_hub->latest_depth()) {
        frame.depth = impl_->sensor_hub->latest_depth()->depth_mm;
    }
    if (impl_->sensor_hub->latest_tof()) {
        perception::TofFrame tof;
        tof.left = impl_->sensor_hub->latest_tof()->left_head;
        tof.right = impl_->sensor_hub->latest_tof()->right_head;
        frame.tof = std::move(tof);
    }
    if (impl_->sensor_hub->latest_pose()) {
        constexpr double kRadiansToDegrees = 57.29577951308232;
        const auto& pose = *impl_->sensor_hub->latest_pose();
        frame.pose = competition::PoseSample{
            pose.xyz[0], pose.xyz[1], pose.rpy[2] * kRadiansToDegrees,
            static_cast<std::uint64_t>(pose.timestamp),
            impl_->sensor_hub->latest_pose_sequence()};
    }
    frame.now = std::chrono::steady_clock::now();
    if (!impl_->football_network.empty() && !frame.rgb.empty()) {
        frame.football = perception::detect_football_dnn(frame.rgb, impl_->football_network);
    }
    if (!config_.allow_empty_frames && !contains_any_input(frame)) {
        return instance::SensorRead::not_ready("standard_sensor_waiting_for_input");
    }
    return instance::SensorRead::ready(std::move(frame));
}

}  // namespace doogle::runtime
