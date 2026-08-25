#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <optional>
#include <string>
#include <vector>

#include "service/perception/geometry.hpp"

namespace doogle::perception {

struct DepthBallPose {
    Meters x_body{};
    Meters y_body{};
    double pixel_x{};
    double pixel_y{};
    double pixel_radius{};
    Meters depth{};
    double confidence{};
};

struct TofFrame {
    std::array<float, 64> left{};
    std::array<float, 64> right{};
};

struct TofBallPose {
    Meters x_body{};
    Meters y_body{};
    Centimeters distance{};
    bool push_reached{};
    enum class Side { Left, Right, Dual } side{Side::Dual};
};

struct TofRampObservation {
    bool reached{};
    Centimeters ball_distance{};
    Centimeters ramp_distance{};
};

struct DepthControl {
    double vx{};
    double vy{};
    double wz{};
    double target_x{320.0};
    int valid_rows{};
    int bad_frames{};
};

class DepthSlopeFollower {
public:
    [[nodiscard]] DepthControl update(const cv::Mat& depth);

private:
    std::array<double, 4> last_center_{320.0, 320.0, 320.0, 320.0};
    std::array<double, 4> last_width_{};
    std::array<bool, 4> has_width_{};
    double target_{320.0};
    double previous_wz_{};
    int bad_frames_{};
};

[[nodiscard]] std::optional<DepthBallPose> calculate_depth_ball_pose(
    const cv::Mat& depth, const cv::Vec3d& rgb_circle, const cv::Size& rgb_size,
    const CameraModel& camera = {388.6661383628198, 388.1784472598178,
                                 325.11847717632475, 233.30839177331623});
[[nodiscard]] std::optional<TofBallPose> process_tof_ball(const TofFrame& frame);
[[nodiscard]] TofRampObservation evaluate_tof_ramp(const TofFrame& frame);

}  // namespace doogle::perception
