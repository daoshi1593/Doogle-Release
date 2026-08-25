#include "service/perception/vision.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace doogle::perception {
namespace {

cv::Mat hsv_mask(const cv::Mat& bgr, const cv::Scalar& low, const cv::Scalar& high) {
    if (bgr.empty()) return {};
    cv::Mat blurred;
    cv::GaussianBlur(bgr, blurred, {5, 5}, 0.0);
    cv::Mat hsv;
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, low, high, mask);
    return mask;
}

cv::Mat clean_mask(cv::Mat mask, int size = 3, int iterations = 1) {
    if (mask.empty()) return mask;
    const auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {size, size});
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, {}, iterations);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, {}, iterations);
    return mask;
}

cv::Mat red_mask(const cv::Mat& bgr) {
    auto low = hsv_mask(bgr, {0, 80, 70}, {10, 255, 255});
    auto high = hsv_mask(bgr, {165, 80, 70}, {179, 255, 255});
    if (low.empty()) return low;
    return clean_mask(low | high, 5);
}

double clip01(double value) { return std::clamp(value, 0.0, 1.0); }

double x_at_y(const cv::Vec4f& line, double y) {
    const double vx = std::abs(line[0]) < 1e-6 ? 1e-6 : line[0];
    const double vy = std::abs(line[1]) < 1e-6 ? 1e-6 : line[1];
    return line[2] + (y - line[3]) * vx / vy;
}

struct FittedLine {
    cv::Vec4f line{};
    double near_x{};
    double far_x{};
    double area{};
};

std::vector<FittedLine> fitted_components(const cv::Mat& mask, double near_y, double far_y,
                                          int min_area, double min_height_ratio) {
    cv::Mat labels, stats, centers;
    const int count = cv::connectedComponentsWithStats(mask, labels, stats, centers, 8);
    std::vector<FittedLine> output;
    for (int label = 1; label < count; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        if (area < min_area || height < static_cast<int>(mask.rows * min_height_ratio)) continue;
        std::vector<cv::Point> points;
        cv::findNonZero(labels == label, points);
        if (points.size() < 8) continue;
        cv::Vec4f line;
        cv::fitLine(points, line, cv::DIST_L2, 0.0, 0.01, 0.01);
        const double slope = std::abs(line[0] / (std::abs(line[1]) < 1e-6 ? 1e-6 : line[1]));
        if (slope > 3.0) continue;
        output.push_back({line, x_at_y(line, near_y), x_at_y(line, far_y), static_cast<double>(area)});
    }
    std::sort(output.begin(), output.end(), [](const auto& a, const auto& b) { return a.near_x < b.near_x; });
    return output;
}

}  // namespace

cv::Mat recover_ball_mask(const cv::Mat& input) {
    if (input.empty()) return {};
    cv::Mat source;
    if (input.type() == CV_8UC1) source = input.clone();
    else input.convertTo(source, CV_8UC1);
    cv::Mat output = source.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(source.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    const double minimum_area = std::max(12.0, static_cast<double>(source.total()) * 0.0005);
    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < minimum_area) continue;
        const auto box = cv::boundingRect(contour);
        cv::Point2f center;
        float radius{};
        const double aspect = static_cast<double>(box.width) / std::max(1, box.height);
        if (aspect >= 1.15) {
            // A flat upper edge is the common half-ball occlusion: the visible
            // component is the lower semicircle, so width and bottom support
            // recover the missing upper half without imposing a maximum radius.
            center = {box.x + box.width * 0.5F, static_cast<float>(box.y)};
            radius = box.width * 0.5F;
        } else {
            cv::minEnclosingCircle(contour, center, radius);
        }
        if (radius < 6.0F) continue;
        cv::Mat candidate(source.size(), CV_8UC1, cv::Scalar{0});
        cv::circle(candidate, center, static_cast<int>(std::ceil(radius)), cv::Scalar{255}, -1);
        cv::Mat supported;
        cv::bitwise_and(candidate, source, supported);
        const double visible = cv::countNonZero(supported);
        const double expected_lower_support = std::max(1.0, CV_PI * radius * radius * 0.35);
        if (visible < expected_lower_support * 0.08) continue;
        cv::bitwise_or(output, candidate, output);
    }
    return output;
}

cv::Mat mask_for_target(const cv::Mat& bgr, VisualTarget target) {
    switch (target) {
        case VisualTarget::RedBall: return recover_ball_mask(red_mask(bgr));
        case VisualTarget::Cola: return red_mask(bgr);
        case VisualTarget::BlueBall:
            return recover_ball_mask(
                clean_mask(hsv_mask(bgr, {98, 100, 145}, {104, 255, 230}), 5));
        case VisualTarget::BlueCube:
            return clean_mask(hsv_mask(bgr, {90, 50, 35}, {135, 255, 255}), 5);
        case VisualTarget::OrangeBall: return clean_mask(hsv_mask(bgr, {4, 90, 90}, {24, 255, 255}), 5);
        default: return {};
    }
}

ComponentMetrics largest_component(const cv::Mat& mask, int min_area, double max_center_y_ratio,
                                   bool prefer_lower_cluster) {
    ComponentMetrics result;
    if (mask.empty()) return result;
    cv::Mat labels, stats, centers;
    const int count = cv::connectedComponentsWithStats(mask, labels, stats, centers, 8);
    struct Candidate { int label; int area; double cx; double cy; cv::Rect box; };
    std::vector<Candidate> candidates;
    for (int label = 1; label < count; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < min_area) continue;
        const double cx = centers.at<double>(label, 0);
        const double cy = centers.at<double>(label, 1);
        if (cy / mask.rows > max_center_y_ratio) continue;
        candidates.push_back({label, area, cx, cy,
                              {stats.at<int>(label, cv::CC_STAT_LEFT), stats.at<int>(label, cv::CC_STAT_TOP),
                               stats.at<int>(label, cv::CC_STAT_WIDTH), stats.at<int>(label, cv::CC_STAT_HEIGHT)}});
    }
    result.component_count = static_cast<int>(candidates.size());
    if (candidates.empty()) return result;
    if (prefer_lower_cluster && candidates.size() >= 3) {
        std::vector<double> ys;
        for (const auto& item : candidates) ys.push_back(item.cy);
        std::sort(ys.begin(), ys.end());
        const double median_y = ys[ys.size() / 2];
        candidates.erase(std::remove_if(candidates.begin(), candidates.end(), [&](const auto& item) {
            return item.cy < median_y - 0.20 * mask.rows;
        }), candidates.end());
    }
    const auto best = std::max_element(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
        return a.area < b.area;
    });
    result.detected = true;
    result.area_ratio = static_cast<double>(best->area) / static_cast<double>(mask.total());
    result.center_x_ratio = best->cx / mask.cols;
    result.center_y_ratio = best->cy / mask.rows;
    result.width_ratio = static_cast<double>(best->box.width) / mask.cols;
    result.height_ratio = static_cast<double>(best->box.height) / mask.rows;
    result.bounds = best->box;
    return result;
}

ColorDetection detect_colored_target(const cv::Mat& bgr, VisualTarget target, double max_center_y_ratio) {
    ColorDetection output;
    output.target = target;
    auto mask = mask_for_target(bgr, target);
    output.component = largest_component(mask, std::max(20, static_cast<int>(mask.total() * 0.001)),
                                         max_center_y_ratio, target == VisualTarget::BlueBall);
    if (!output.component.detected) return output;
    output.reason = "detected";
    output.confidence = clip01(output.component.area_ratio / 0.04);
    const double error = output.component.center_x_ratio - 0.5;
    output.correction = std::abs(error) <= 0.03 ? "done" : (error < 0.0 ? "left" : "right");
    return output;
}

YellowLineObservation analyze_yellow_line(const cv::Mat& bgr, double roi_top_ratio) {
    YellowLineObservation result;
    if (bgr.empty()) return result;
    auto mask = clean_mask(hsv_mask(bgr, {18, 80, 90}, {38, 255, 255}), 3, 2);
    const int top = std::clamp(static_cast<int>(bgr.rows * roi_top_ratio), 0, bgr.rows - 1);
    mask.rowRange(0, top).setTo(0);
    cv::Mat labels, stats, centers;
    const int count = cv::connectedComponentsWithStats(mask, labels, stats, centers, 8);
    int best = -1;
    int best_area = 0;
    for (int label = 1; label < count; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area > best_area && area >= std::max(12, static_cast<int>(mask.total() * 0.00008))) {
            best = label;
            best_area = area;
        }
    }
    if (best < 0) return result;
    std::vector<cv::Point> points;
    cv::findNonZero(labels == best, points);
    if (points.size() < 8) return result;
    cv::Vec4f line;
    cv::fitLine(points, line, cv::DIST_L2, 0.0, 0.01, 0.01);
    const auto bottom = *std::max_element(points.begin(), points.end(), [](const auto& a, const auto& b) {
        return a.y < b.y;
    });
    result.detected = true;
    result.bottom_x = bottom.x;
    result.bottom_y = bottom.y;
    result.angle_deg = std::atan2(line[1], line[0]) * 180.0 / CV_PI;
    while (result.angle_deg > 90.0) result.angle_deg -= 180.0;
    while (result.angle_deg < -90.0) result.angle_deg += 180.0;
    const double center_x = bgr.cols * 0.5;
    const double line_x = x_at_y(line, bgr.rows * 0.75);
    result.centerline_gap = std::abs(line_x - center_x);
    const int min_x = std::min_element(points.begin(), points.end(), [](const auto& a, const auto& b) {
        return a.x < b.x;
    })->x;
    const int max_x = std::max_element(points.begin(), points.end(), [](const auto& a, const auto& b) {
        return a.x < b.x;
    })->x;
    result.crosses_centerline = min_x <= center_x && max_x >= center_x;
    result.reason = result.crosses_centerline ? "centerline_tangent" : "nearest_to_image_centerline";
    return result;
}

LaneObservation detect_three_line_lane(const cv::Mat& bgr) {
    LaneObservation result;
    if (bgr.empty()) return result;
    auto mask = clean_mask(hsv_mask(bgr, {15, 65, 85}, {42, 255, 255}), 5);
    const int top = static_cast<int>(bgr.rows * 0.38);
    mask.rowRange(0, top).setTo(0);
    auto lines = fitted_components(mask, bgr.rows * 0.88, bgr.rows * 0.55, 80, 0.035);
    result.line_count = static_cast<int>(lines.size());
    if (lines.size() < 3) return result;
    const auto& left = lines[lines.size() - 2];
    const auto& right = lines[lines.size() - 1];
    result.detected = true;
    result.reason = "right_two_of_three";
    result.left_near_x = left.near_x;
    result.right_near_x = right.near_x;
    result.left_far_x = left.far_x;
    result.right_far_x = right.far_x;
    const double near_center = 0.5 * (left.near_x + right.near_x);
    const double far_center = 0.5 * (left.far_x + right.far_x);
    result.center_error = (near_center - bgr.cols * 0.5) / (bgr.cols * 0.5);
    result.heading_error = std::atan2(near_center - far_center, bgr.rows * (0.88 - 0.55));
    return result;
}

MarkerObservation detect_bottom_yellow_marker(const cv::Mat& bgr, double roi_ratio, double threshold) {
    MarkerObservation result{false, 0.0, threshold};
    if (bgr.empty()) return result;
    auto mask = hsv_mask(bgr, {15, 65, 85}, {42, 255, 255});
    const int top = std::clamp(static_cast<int>(bgr.rows * (1.0 - roi_ratio)), 0, bgr.rows - 1);
    const auto roi = mask.rowRange(top, bgr.rows);
    result.fill_ratio = static_cast<double>(cv::countNonZero(roi)) / roi.total();
    result.detected = result.fill_ratio >= threshold;
    return result;
}

MarkerObservation detect_top_red_marker(const cv::Mat& bgr, double top_ratio, double threshold) {
    MarkerObservation result{false, 0.0, threshold};
    if (bgr.empty()) return result;
    auto mask = red_mask(bgr);
    const int bottom = std::clamp(static_cast<int>(bgr.rows * top_ratio), 1, bgr.rows);
    const auto roi = mask.rowRange(0, bottom);
    result.fill_ratio = static_cast<double>(cv::countNonZero(roi)) / roi.total();
    result.detected = result.fill_ratio >= threshold;
    return result;
}

LimitBarObservation detect_limit_bar(const cv::Mat& bgr) {
    LimitBarObservation result;
    if (bgr.empty()) return result;
    const auto red = red_mask(bgr);
    cv::Mat horizontal, vertical;
    cv::morphologyEx(red, horizontal, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, {31, 5}));
    cv::morphologyEx(red, vertical, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, {5, 31}));
    int band_start = -1;
    int best_start = -1;
    int best_end = -1;
    for (int row = 0; row < bgr.rows; ++row) {
        const bool horizontal_row = cv::countNonZero(horizontal.row(row)) > bgr.cols * 0.35;
        if (horizontal_row && band_start < 0) band_start = row;
        if ((!horizontal_row || row == bgr.rows - 1) && band_start >= 0) {
            const int band_end = horizontal_row ? row + 1 : row;
            if (band_end - band_start > best_end - best_start) {
                best_start = band_start;
                best_end = band_end;
            }
            band_start = -1;
        }
    }
    if (best_start >= 0 && best_end > best_start && best_start < bgr.rows * 0.70) {
        std::vector<cv::Point> beam_points;
        cv::findNonZero(red.rowRange(best_start, best_end), beam_points);
        if (!beam_points.empty()) {
            result.beam = cv::boundingRect(beam_points);
            result.beam.y += best_start;
        }
    }
    if (result.beam.empty()) return result;
    std::vector<std::vector<cv::Point>> contours;
    const int post_top = std::min(bgr.rows - 1, result.beam.y + result.beam.height);
    cv::Mat post_region = vertical.rowRange(post_top, bgr.rows).clone();
    cv::findContours(post_region, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        auto box = cv::boundingRect(contour);
        box.y += post_top;
        if (box.height > bgr.rows * 0.18 && box.width < bgr.cols * 0.18 &&
            box.y <= result.beam.y + result.beam.height * 2) result.posts.push_back(box);
    }
    const bool left = std::any_of(result.posts.begin(), result.posts.end(), [&](const auto& post) {
        return post.x + post.width * 0.5 < result.beam.x + result.beam.width * 0.35;
    });
    const bool right = std::any_of(result.posts.begin(), result.posts.end(), [&](const auto& post) {
        return post.x + post.width * 0.5 > result.beam.x + result.beam.width * 0.65;
    });
    result.detected = left && right;
    result.near = result.beam.y > bgr.rows * 0.45 || result.beam.width > bgr.cols * 0.70;
    result.confidence = clip01(result.beam.width / (bgr.cols * 0.75)) * (result.detected ? 1.0 : 0.5);
    result.reason = result.detected ? "beam_and_posts" : "beam_without_two_posts";
    return result;
}

FootballObservation detect_football_heuristic(const cv::Mat& bgr) {
    FootballObservation best;
    if (bgr.empty()) return best;
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, {7, 7}, 1.5);
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1.2, 20.0, 80.0, 24.0, 6,
                     std::min(bgr.rows, bgr.cols) / 3);
    for (const auto& circle : circles) {
        const int radius = static_cast<int>(std::round(circle[2]));
        cv::Rect box{static_cast<int>(circle[0]) - radius, static_cast<int>(circle[1]) - radius,
                     radius * 2, radius * 2};
        box &= cv::Rect{0, 0, bgr.cols, bgr.rows};
        if (box.empty()) continue;
        const cv::Mat roi = gray(box);
        const double black = static_cast<double>(cv::countNonZero(roi < 95)) / roi.total();
        const double white = static_cast<double>(cv::countNonZero(roi > 150)) / roi.total();
        const double aspect = static_cast<double>(box.width) / std::max(1, box.height);
        const double area_ratio = static_cast<double>(box.area()) / bgr.total();
        const double shape_score = clip01(1.0 - std::abs(aspect - 1.0));
        const double black_score = clip01(black / 0.12);
        const double white_score = clip01(white / 0.30);
        const double area_score = area_ratio >= 0.002 && area_ratio <= 0.16 ? 1.0 : 0.0;
        const double score = 0.30 * shape_score + 0.25 * black_score + 0.25 * white_score + 0.20 * area_score;
        if (score > best.score) {
            best = {score >= 0.68, box, {circle[0], circle[1]}, circle[2], score, black, white,
                    score >= 0.68 ? "football_score_pass" : "football_score_low"};
        }
    }
    return best;
}

FootballObservation detect_football_dnn(const cv::Mat& bgr, cv::dnn::Net& network,
                                        float confidence_threshold, float nms_threshold) {
    FootballObservation result;
    result.reason = "dnn_no_detection";
    if (bgr.empty() || network.empty()) return result;
    constexpr int kInputSize = 640;
    cv::Mat blob;
    cv::dnn::blobFromImage(bgr, blob, 1.0 / 255.0, {kInputSize, kInputSize}, {}, true, false);
    network.setInput(blob);
    std::vector<cv::Mat> outputs;
    network.forward(outputs, network.getUnconnectedOutLayersNames());
    if (outputs.empty() || outputs.front().empty()) {
        result.reason = "dnn_empty_output";
        return result;
    }
    cv::Mat rows;
    auto output = outputs.front();
    if (output.dims == 3) {
        const int first = output.size[1];
        const int second = output.size[2];
        if (first < second) {
            cv::Mat channels(first, second, CV_32F, output.ptr<float>());
            cv::transpose(channels, rows);
        } else {
            rows = cv::Mat(first, second, CV_32F, output.ptr<float>()).clone();
        }
    } else if (output.dims == 2) {
        rows = output;
    } else {
        result.reason = "dnn_unsupported_shape";
        return result;
    }
    if (rows.cols < 5) {
        result.reason = "dnn_unsupported_shape";
        return result;
    }
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    const double scale_x = static_cast<double>(bgr.cols) / kInputSize;
    const double scale_y = static_cast<double>(bgr.rows) / kInputSize;
    for (int row = 0; row < rows.rows; ++row) {
        const auto* values = rows.ptr<float>(row);
        float score = 0.0F;
        for (int column = 4; column < rows.cols; ++column) score = std::max(score, values[column]);
        if (!std::isfinite(score) || score < confidence_threshold) continue;
        const double width = values[2] * scale_x;
        const double height = values[3] * scale_y;
        cv::Rect box{static_cast<int>(std::round(values[0] * scale_x - width * 0.5)),
                     static_cast<int>(std::round(values[1] * scale_y - height * 0.5)),
                     static_cast<int>(std::round(width)), static_cast<int>(std::round(height))};
        box &= cv::Rect{0, 0, bgr.cols, bgr.rows};
        if (!box.empty()) {
            boxes.push_back(box);
            scores.push_back(score);
        }
    }
    std::vector<int> kept;
    cv::dnn::NMSBoxes(boxes, scores, confidence_threshold, nms_threshold, kept);
    if (kept.empty()) return result;
    const int best = *std::max_element(kept.begin(), kept.end(), [&](int left, int right) {
        return scores[static_cast<std::size_t>(left)] < scores[static_cast<std::size_t>(right)];
    });
    const auto best_index = static_cast<std::size_t>(best);
    const auto box = boxes[best_index];
    result.detected = true;
    result.box = box;
    result.center = {box.x + box.width * 0.5, box.y + box.height * 0.5};
    result.radius = 0.25 * (box.width + box.height);
    result.score = scores[best_index];
    result.reason = "football_dnn";
    return result;
}

bool detect_yellow_boundary(const cv::Mat& bgr, double* mean_y_norm, double* slope) {
    if (bgr.empty()) return false;
    auto mask = clean_mask(hsv_mask(bgr, {15, 75, 90}, {35, 255, 255}), 5);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> valid;
    for (const auto& contour : contours) {
        const auto box = cv::boundingRect(contour);
        if (cv::contourArea(contour) >= 1000.0 &&
            static_cast<double>(box.width) / std::max(1, box.height) >= 2.5) valid.push_back(contour);
    }
    if (valid.size() != 1) return false;
    const auto moments = cv::moments(valid.front());
    if (moments.m00 == 0.0) return false;
    const double y = moments.m01 / moments.m00 / bgr.rows;
    cv::Vec4f line;
    cv::fitLine(valid.front(), line, cv::DIST_L2, 0.0, 0.01, 0.01);
    const double k = line[1] / (std::abs(line[0]) < 1e-6 ? 1e-6 : line[0]);
    if (mean_y_norm) *mean_y_norm = y;
    if (slope) *slope = k;
    return y >= 0.50 && std::abs(k) <= 0.18;
}

double blue_roi_ratio(const cv::Mat& bgr, const cv::Rect2d& normalized_roi) {
    if (bgr.empty()) return 0.0;
    const int x0 = std::clamp(static_cast<int>(normalized_roi.x * bgr.cols), 0, bgr.cols - 1);
    const int y0 = std::clamp(static_cast<int>(normalized_roi.y * bgr.rows), 0, bgr.rows - 1);
    const int x1 = std::clamp(static_cast<int>((normalized_roi.x + normalized_roi.width) * bgr.cols), x0 + 1, bgr.cols);
    const int y1 = std::clamp(static_cast<int>((normalized_roi.y + normalized_roi.height) * bgr.rows), y0 + 1, bgr.rows);
    const auto mask = mask_for_target(bgr, VisualTarget::BlueBall);
    const auto roi = mask(cv::Rect{x0, y0, x1 - x0, y1 - y0});
    return static_cast<double>(cv::countNonZero(roi)) / roi.total();
}

MotionCorrection compute_s_curve_command(const YellowLineObservation& left,
                                         const YellowLineObservation& right, double speed) {
    MotionCorrection output;
    output.vx = speed;
    if (left.detected && right.detected) {
        if (left.angle_deg * right.angle_deg < 0.0 &&
            std::abs(left.angle_deg - right.angle_deg) > 20.0) {
            output.hold_yaw = true;
            return output;
        }
        const double gap_error = std::clamp((left.bottom_x - right.bottom_x) / 640.0, -0.20, 0.20);
        output.vy = std::clamp(-0.50 * gap_error, -0.10, 0.10);
        output.wz = std::clamp(-0.5 * (left.angle_deg + right.angle_deg) * CV_PI / 180.0,
                               -0.22, 0.22);
    } else if (right.detected) {
        output.wz = std::clamp(-0.18 * right.angle_deg, -0.22, 0.22);
    } else if (left.detected) {
        output.wz = std::clamp(-0.18 * left.angle_deg, -0.22, 0.22);
    } else {
        output.vx = 0.04;
    }
    return output;
}

}  // namespace doogle::perception
