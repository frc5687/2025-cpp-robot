
#pragma once

#include <frc/Timer.h>
#include <frc/controller/PIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Rotation2d.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <functional>
#include <optional>

#include "subsystem/drive/DriveSubsystem.h"

class DriveMaintainingHeadingCommand
    : public frc2::CommandHelper<frc2::Command,
                                 DriveMaintainingHeadingCommand> {
public:
  DriveMaintainingHeadingCommand(DriveSubsystem *driveSubsystem,
                                 std::function<double()> throttle,
                                 std::function<double()> strafe,
                                 std::function<double()> turn,
                                 bool enableSlewRate = true);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  void SetHeadingPID(double kP, double kI, double kD);

private:
  DriveSubsystem *m_driveSubsystem;
  std::function<double()> m_throttleSupplier;
  std::function<double()> m_strafeSupplier;
  std::function<double()> m_turnSupplier;

  bool m_enableSlewRate;
  std::optional<frc::Rotation2d> m_headingSetpoint;
  double m_joystickLastTouched;

  // We are currently just limiting the joysticks, not the MPS of robot
  // If we want to later, change to MPS and limit velocity not joysticks input
  // TODO: move to constants
  frc::SlewRateLimiter<units::scalar> m_xLimiter{3.0 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yLimiter{3.0 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3.0 / 1_s};

  // TODO: move to constants
  frc::PIDController m_headingController{5.0, 0.0, 0.0};

  double ApplyDeadband(double value, double deadband);

  units::radians_per_second_t
  CalculateHeadingCorrection(frc::Rotation2d current, frc::Rotation2d target);

  bool IsTurnInputActive(double turnInput, double deadband);
};
