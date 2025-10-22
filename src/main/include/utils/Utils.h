
#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <numbers>

constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kTwoPi = 2.0 * kPi;

// Wrap radians into [-pi, pi]
inline units::radian_t WrapToPi(units::radian_t a) {
  return units::radian_t{std::remainder(a.value(), kTwoPi)};
}

// Wrap turns to (-0.5, 0.5] turns (i.e., [-pi, pi] when converted to radians)
inline units::turn_t WrapToHalfTurns(units::turn_t t) {
  return units::turn_t{std::remainder(t.value(), 1.0)};
}

// Wheel-side: rad/s -> m/s (cancel angle dimension explicitly)
inline units::meters_per_second_t WheelOmegaToMps(
    units::radians_per_second_t wheel_omega, units::meter_t wheel_radius) {
  return (wheel_omega / 1_rad) * wheel_radius;  // (rad/s)/rad * m = m/s
}
