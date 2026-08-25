#include "service/perception/geometry.hpp"

#include <cmath>

namespace doogle::perception {

double normalize_angle_degrees(double value) {
    while (value > 180.0) value -= 360.0;
    while (value < -180.0) value += 360.0;
    return value;
}

std::optional<BallGeometry> camera_ball_to_body(const CameraModel& camera, double pixel_x,
                                                double pixel_y, double pixel_radius, Meters depth,
                                                Meters x_offset, Meters y_offset) {
    if (camera.fx <= 0.0 || camera.fy <= 0.0 || depth.value <= 0.0 || pixel_radius <= 0.0) return std::nullopt;
    const double x_camera = (pixel_x - camera.cx) * depth.value / camera.fx;
    const double y_camera = (pixel_y - camera.cy) * depth.value / camera.fy;
    const double focal_mean = std::sqrt(camera.fx * camera.fy);
    return BallGeometry{{{depth.value + x_offset.value}, {x_offset.value - x_camera + y_offset.value},
                         {y_camera}}, {pixel_radius * depth.value / focal_mean}};
}

std::optional<Meters> tof_distance(Centimeters distance, Centimeters min_valid,
                                   Centimeters max_valid) {
    if (distance.value < min_valid.value || distance.value > max_valid.value) return std::nullopt;
    return to_meters(distance);
}

}  // namespace doogle::perception
