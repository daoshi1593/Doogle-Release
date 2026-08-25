#pragma once

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include <optional>
#include <string>
#include <vector>

namespace doogle::perception {

enum class VisualTarget { None, RedBall, BlueBall, OrangeBall, Football, Cola, BlueCube };

struct ComponentMetrics {
    bool detected{};
    int component_count{};
    double area_ratio{};
    double center_x_ratio{};
    double center_y_ratio{};
    double width_ratio{};
    double height_ratio{};
    cv::Rect bounds{};
};

struct ColorDetection {
    VisualTarget target{VisualTarget::None};
    ComponentMetrics component{};
    double confidence{};
    std::string reason{"no_color_component"};
    std::string correction{"none"};
};

struct YellowLineObservation {
    bool detected{};
    double bottom_x{};
    double bottom_y{};
    double angle_deg{};
    double centerline_gap{};
    bool crosses_centerline{};
    std::string reason{"no_yellow_component"};
};

struct LaneObservation {
    bool detected{};
    int line_count{};
    double left_near_x{};
    double right_near_x{};
    double left_far_x{};
    double right_far_x{};
    double center_error{};
    double heading_error{};
    std::string reason{"need_three_yellow_lines"};
};

struct MarkerObservation {
    bool detected{};
    double fill_ratio{};
    double threshold{};
};

struct LimitBarObservation {
    bool detected{};
    bool near{};
    double confidence{};
    cv::Rect beam{};
    std::vector<cv::Rect> posts{};
    std::string reason{"no_red_beam"};
};

struct FootballObservation {
    bool detected{};
    cv::Rect box{};
    cv::Point2d center{};
    double radius{};
    double score{};
    double black_ratio{};
    double white_ratio{};
    std::string reason{"no_football_candidate"};
};

struct MotionCorrection {
    double vx{};
    double vy{};
    double wz{};
    bool hold_yaw{};
};

[[nodiscard]] cv::Mat mask_for_target(const cv::Mat& bgr, VisualTarget target);
[[nodiscard]] cv::Mat recover_ball_mask(const cv::Mat& mask);
[[nodiscard]] ComponentMetrics largest_component(const cv::Mat& mask, int min_area = 12,
                                                 double max_center_y_ratio = 1.0,
                                                 bool prefer_lower_cluster = false);
[[nodiscard]] ColorDetection detect_colored_target(const cv::Mat& bgr, VisualTarget target,
                                                   double max_center_y_ratio = 0.78);
[[nodiscard]] YellowLineObservation analyze_yellow_line(const cv::Mat& bgr,
                                                        double roi_top_ratio = 0.75);
[[nodiscard]] LaneObservation detect_three_line_lane(const cv::Mat& bgr);
[[nodiscard]] MarkerObservation detect_bottom_yellow_marker(const cv::Mat& bgr,
                                                            double roi_ratio = 0.10,
                                                            double threshold = 0.30);
[[nodiscard]] MarkerObservation detect_top_red_marker(const cv::Mat& bgr,
                                                      double top_ratio = 0.20,
                                                      double threshold = 0.50);
[[nodiscard]] LimitBarObservation detect_limit_bar(const cv::Mat& bgr);
[[nodiscard]] FootballObservation detect_football_heuristic(const cv::Mat& bgr);
[[nodiscard]] FootballObservation detect_football_dnn(const cv::Mat& bgr, cv::dnn::Net& network,
                                                      float confidence_threshold = 0.35F,
                                                      float nms_threshold = 0.45F);
[[nodiscard]] bool detect_yellow_boundary(const cv::Mat& bgr, double* mean_y_norm = nullptr,
                                          double* slope = nullptr);
[[nodiscard]] double blue_roi_ratio(const cv::Mat& bgr, const cv::Rect2d& normalized_roi);
[[nodiscard]] MotionCorrection compute_s_curve_command(const YellowLineObservation& left,
                                                       const YellowLineObservation& right,
                                                       double speed = 0.15);

}  // namespace doogle::perception
