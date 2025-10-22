
#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/temperature.h>

struct GyroIOInputs {
  frc::Rotation2d yaw{0_rad};
  units::radians_per_second_t yawRate{0};
  units::radians_per_second_squared_t yawAcceleration{0};

  // Optional 3D orientation (for advanced IMUs)
  frc::Rotation2d pitch{0_rad};
  frc::Rotation2d roll{0_rad};

  // Status
  bool connected{true};
  units::celsius_t temperature{25};

  // For logging
  double timestamp{0};
};

class GyroIO {
 public:
  virtual ~GyroIO() = default;

  virtual void UpdateInputs(GyroIOInputs& inputs, bool isBatched) = 0;
  virtual void Reset(units::degree_t angle = 0_deg) = 0;

  virtual void UpdateWithOdometry(const frc::ChassisSpeeds& robotSpeeds) {
    // Default implementation does nothing, only SimGyroIO overrides this
  }
};
