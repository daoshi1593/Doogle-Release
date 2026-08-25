#pragma once

#include <cmath>

namespace doogle {

struct Meters {
    double value{};
    friend constexpr bool operator==(Meters, Meters) = default;
};
struct Centimeters {
    double value{};
    friend constexpr bool operator==(Centimeters, Centimeters) = default;
};
struct Radians {
    double value{};
    friend constexpr bool operator==(Radians, Radians) = default;
};
struct Degrees {
    double value{};
    friend constexpr bool operator==(Degrees, Degrees) = default;
};

constexpr Meters to_meters(Centimeters value) { return {value.value / 100.0}; }
constexpr Centimeters to_centimeters(Meters value) { return {value.value * 100.0}; }
constexpr Radians to_radians(Degrees value) { return {value.value * 3.14159265358979323846 / 180.0}; }
constexpr Degrees to_degrees(Radians value) { return {value.value * 180.0 / 3.14159265358979323846}; }

}  // namespace doogle
