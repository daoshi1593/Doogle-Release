#pragma once

#include <optional>

#include "service/domain/units.hpp"

namespace doogle::perception {

struct CameraModel {
    double fx{};
    double fy{};
    double cx{};
    double cy{};
};

struct BodyPoint {
    Meters x;
    Meters y;
    Meters z;
};

struct BallGeometry {
    BodyPoint body;
    Meters radius;
};

[[nodiscard]] double normalize_angle_degrees(double value);
[[nodiscard]] std::optional<BallGeometry> camera_ball_to_body(
    const CameraModel&, double pixel_x, double pixel_y, double pixel_radius, Meters depth,
    Meters x_offset, Meters y_offset);
[[nodiscard]] std::optional<Meters> tof_distance(Centimeters distance, Centimeters min_valid,
                                                 Centimeters max_valid);

}  // namespace doogle::perception
