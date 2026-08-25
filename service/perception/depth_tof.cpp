#include "service/perception/depth_tof.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace doogle::perception {
namespace {

constexpr std::array<int, 4> kRows{280, 320, 360, 400};
constexpr std::array<double, 4> kWeights{0.50, 0.30, 0.15, 0.05};

std::optional<double> valid_median(std::vector<double> values) {
    values.erase(std::remove_if(values.begin(), values.end(), [](double value) { return value <= 0.0; }),
                 values.end());
    if (values.size() < 2) return std::nullopt;
    const auto middle = values.begin() + static_cast<long>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

double depth_value(const cv::Mat& depth, int row, int column) {
    switch (depth.type()) {
        case CV_16UC1: return depth.at<std::uint16_t>(row, column);
        case CV_32FC1: return depth.at<float>(row, column);
        case CV_64FC1: return depth.at<double>(row, column);
        default: return 0.0;
    }
}

double edge_score(const cv::Mat& depth, int row, int column, int gap = 3, int window = 5) {
    std::vector<double> left, right;
    for (int x = std::max(0, column - gap - window); x < column - gap; ++x)
        left.push_back(depth_value(depth, row, x));
    for (int x = column + gap; x < std::min(depth.cols, column + gap + window); ++x)
        right.push_back(depth_value(depth, row, x));
    const auto left_median = valid_median(left);
    const auto right_median = valid_median(right);
    if (!left_median && !right_median) return 0.0;
    if (left_median.has_value() != right_median.has_value()) return 999.0;
    return std::abs(*right_median - *left_median);
}

std::optional<int> find_edge(const cv::Mat& depth, int row, double seed, int direction) {
    int consecutive = 0;
    const int begin = static_cast<int>(seed) + direction * 15;
    for (int x = begin; x > 50 && x < depth.cols - 50; x += direction) {
        if (edge_score(depth, row, x) > 70.0) {
            if (++consecutive >= 2) return x;
        } else {
            consecutive = 0;
        }
    }
    return std::nullopt;
}

std::array<double, 64> centimeters(const std::array<float, 64>& input, bool flip_vertical) {
    std::array<double, 64> output{};
    const double maximum = *std::max_element(input.begin(), input.end());
    const double scale = maximum <= 2.0 ? 100.0 : (maximum > 200.0 ? 0.1 : 1.0);
    for (int row = 0; row < 8; ++row) {
        const int source_row = flip_vertical ? 7 - row : row;
        for (int col = 0; col < 8; ++col) output[row * 8 + col] = input[source_row * 8 + col] * scale;
    }
    return output;
}

double low_quantile(std::vector<double> values) {
    if (values.empty()) return 99.0;
    std::sort(values.begin(), values.end());
    if (values.size() < 3) return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    return values[static_cast<std::size_t>(std::floor(0.20 * (values.size() - 1)))];
}

}  // namespace

DepthControl DepthSlopeFollower::update(const cv::Mat& depth) {
    DepthControl output;
    if (depth.empty() || depth.channels() != 1) return output;
    std::vector<double> targets;
    std::vector<double> weights;
    for (std::size_t index = 0; index < kRows.size(); ++index) {
        const int row = kRows[index];
        if (row >= depth.rows) continue;
        const auto left = find_edge(depth, row, last_center_[index], -1);
        const auto right = find_edge(depth, row, last_center_[index], 1);
        std::optional<double> target;
        if (left && right) {
            double width = *right - *left;
            if (width > 80.0 && width < depth.cols - 60.0) {
                if (has_width_[index]) width = 0.8 * last_width_[index] + 0.2 * width;
                last_width_[index] = width;
                has_width_[index] = true;
                last_center_[index] = 0.5 * (*left + *right);
                target = *left + 0.47 * width;
            }
        } else if (has_width_[index]) {
            if (left) target = *left + 0.47 * last_width_[index];
            if (right) target = *right - 0.53 * last_width_[index];
        }
        if (target) {
            targets.push_back(*target);
            weights.push_back(kWeights[index]);
        }
    }
    output.valid_rows = static_cast<int>(targets.size());
    if (targets.size() < 2) {
        output.bad_frames = ++bad_frames_;
        if (bad_frames_ < 3) {
            output.vx = 0.04;
            output.wz = previous_wz_;
        }
        return output;
    }
    bad_frames_ = 0;
    const double weight_sum = std::accumulate(weights.begin(), weights.end(), 0.0);
    double raw_target = 0.0;
    for (std::size_t i = 0; i < targets.size(); ++i) raw_target += targets[i] * weights[i] / weight_sum;
    target_ = 0.70 * target_ + 0.30 * raw_target;
    const double error = (target_ - 320.0) / 320.0;
    const double raw_wz = std::clamp(-1.2 * error, -0.45, 0.45);
    output.wz = 0.60 * previous_wz_ + 0.40 * raw_wz;
    previous_wz_ = output.wz;
    output.vx = std::abs(error) < 0.08 ? 0.15 : (std::abs(error) < 0.18 ? 0.10 : 0.06);
    output.target_x = target_;
    output.bad_frames = 0;
    return output;
}

std::optional<DepthBallPose> calculate_depth_ball_pose(const cv::Mat& depth,
                                                       const cv::Vec3d& rgb_circle,
                                                       const cv::Size& rgb_size,
                                                       const CameraModel& camera) {
    if (depth.empty() || rgb_size.empty() || rgb_circle[2] <= 0.0) return std::nullopt;
    const double sx = static_cast<double>(depth.cols) / rgb_size.width;
    const double sy = static_cast<double>(depth.rows) / rgb_size.height;
    const int cx = static_cast<int>(std::round(rgb_circle[0] * sx));
    const int cy = static_cast<int>(std::round(rgb_circle[1] * sy));
    const int radius = std::max(3, static_cast<int>(std::round(rgb_circle[2] * sx)));
    const int inner = std::max(3, static_cast<int>(radius * 0.45));
    std::vector<double> values;
    for (int y = std::max(0, cy - inner); y < std::min(depth.rows, cy + inner); ++y) {
        for (int x = std::max(0, cx - inner); x < std::min(depth.cols, cx + inner); ++x) {
            double value = depth_value(depth, y, x);
            if (depth.type() == CV_16UC1 || value > 20.0) value /= 1000.0;
            if (std::isfinite(value) && value >= 0.20 && value <= 2.50) values.push_back(value);
        }
    }
    if (values.size() < 6) return std::nullopt;
    const double z = *valid_median(values);
    const double physical_radius = rgb_circle[2] * z / std::sqrt(camera.fx * camera.fy);
    if (physical_radius < 0.050 || physical_radius > 0.175) return std::nullopt;
    const double x_camera = (rgb_circle[0] - camera.cx) * z / camera.fx;
    return DepthBallPose{{z + 0.20}, {-x_camera + 0.035}, rgb_circle[0], rgb_circle[1],
                         rgb_circle[2], {z}, 0.90};
}

std::optional<TofBallPose> process_tof_ball(const TofFrame& frame) {
    const auto left = centimeters(frame.left, false);
    const auto right = centimeters(frame.right, true);
    std::vector<double> left_valid, right_valid;
    for (int row = 4; row < 8; ++row) {
        for (int col = 0; col < 8; ++col) {
            const double l = left[row * 8 + col];
            const double r = right[row * 8 + col];
            if (l > 0.0 && l <= 32.0) left_valid.push_back(l);
            if (r > 0.0 && r <= 32.0) right_valid.push_back(r);
        }
    }
    const double dl = low_quantile(left_valid);
    const double dr = low_quantile(right_valid);
    const double minimum = std::min(dl, dr);
    if (minimum > 25.0) return std::nullopt;
    TofBallPose result;
    if (dl <= 25.0 && dr <= 25.0) {
        result.side = TofBallPose::Side::Dual;
        result.y_body = {-0.0375 * (dl - dr) + 0.035};
    } else {
        const bool use_left = dl <= 25.0;
        result.side = use_left ? TofBallPose::Side::Left : TofBallPose::Side::Right;
        const auto& matrix = use_left ? left : right;
        double weighted_column = 0.0;
        double weight_sum = 0.0;
        for (int row = 4; row < 8; ++row) {
            for (int col = 0; col < 8; ++col) {
                const double value = matrix[row * 8 + col];
                if (value > 0.0 && value <= 32.0) {
                    const double weight = 1.0 / std::max(value, 1.0);
                    weighted_column += col * weight;
                    weight_sum += weight;
                }
            }
        }
        const double center = weight_sum > 0.0 ? weighted_column / weight_sum : 3.5;
        if (use_left) result.y_body = {0.065 + 0.010 * (center - 3.5) + 0.008 * (dl - 15.0) + 0.035};
        else result.y_body = {-0.065 - 0.010 * (3.5 - center) - 0.008 * (dr - 15.0) + 0.035};
    }
    const double local_y = result.y_body.value +
        (result.side == TofBallPose::Side::Left ? -0.065 :
         result.side == TofBallPose::Side::Right ? 0.065 : 0.0);
    const double decoupled = std::sqrt(std::max(minimum * minimum - std::pow(local_y * 100.0, 2),
                                                12.8 * 12.8));
    result.distance = {decoupled};
    result.push_reached = decoupled <= 16.0;
    if (result.push_reached) result.x_body = {0.20};
    else result.x_body = {(2.25 * (decoupled - 14.1) + 12.0) / 100.0 + 0.20};
    return result;
}

TofRampObservation evaluate_tof_ramp(const TofFrame& frame) {
    const auto left = centimeters(frame.left, false);
    const auto right = centimeters(frame.right, true);
    double ball = 99.0;
    double environment_sum = 0.0;
    int environment_count = 0;
    for (int row = 4; row < 8; ++row)
        for (int col = 0; col < 4; ++col) ball = std::min({ball, left[row * 8 + col], right[row * 8 + col]});
    for (int row = 6; row < 8; ++row) {
        for (int col = 6; col < 8; ++col) {
            environment_sum += left[row * 8 + col] + right[row * 8 + col];
            environment_count += 2;
        }
    }
    const double ramp = environment_count ? environment_sum / environment_count : 99.0;
    return {ball <= 18.0 && ramp <= 27.0, {ball}, {ramp}};
}

}  // namespace doogle::perception
