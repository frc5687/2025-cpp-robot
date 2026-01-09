#pragma once

#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>

#include <frc/system/plant/DCMotor.h>

namespace Constants::Flywheel {
// Motor constants
inline constexpr bool kMotorInverted = false;
inline constexpr double kGearRatio = 1.0 / 1.25; // this is a 1.25 increase in speed not a reduction

inline constexpr units::meter_t kFlywheelRadius = 2_in;
inline constexpr units::kilogram_t kFlywheelMass = 0.3_kg;

// 1/2 MR²
inline constexpr units::kilogram_square_meter_t kFlywheelMOI = 0.5 * kFlywheelMass * kFlywheelRadius * kFlywheelRadius;

inline constexpr int kNumMotors = 2;
inline constexpr frc::DCMotor kMotor = frc::DCMotor::KrakenX60FOC(kNumMotors);


} /* Constants::Flywheel */
