
#pragma once
#include <frc/geometry/Translation2d.h>
#include <frc/system/plant/DCMotor.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/apriltag/AprilTagFields.h"

namespace Constants {

inline constexpr units::second_t kLoopPeriod = 20_ms;
inline constexpr units::second_t kLogPeriod = 100_ms;
inline constexpr units::volt_t kNominalVoltage = 12_V;
inline constexpr double kVoltageCompensation = 12.0;
inline constexpr double kJoystickDeadband = 0.1;

namespace Field {
inline const frc::AprilTagFieldLayout kFieldTagLayout =
    frc::AprilTagFieldLayout::LoadField(
        frc::AprilTagField::k2025ReefscapeAndyMark);
}  // namespace Field

}  // namespace Constants
