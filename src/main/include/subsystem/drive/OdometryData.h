
#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <units/time.h>

#include <array>

#include "SwerveConstants.h"

struct OdometryData {
  frc::Pose2d pose{};
  std::array<frc::SwerveModulePosition, Constants::SwerveDrive::kModuleCount>
      modulePositions{};
  std::array<frc::SwerveModuleState, Constants::SwerveDrive::kModuleCount>
      moduleStates{};
  frc::ChassisSpeeds chassisSpeeds{};
  frc::Rotation2d gyroAngle{};
  units::second_t timestamp{0_s};
  bool isValid{false};
};
