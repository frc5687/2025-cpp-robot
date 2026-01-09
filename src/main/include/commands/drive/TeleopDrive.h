
#pragma once

#include <frc/filter/SlewRateLimiter.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/acceleration.h>

#include <functional>

#include "subsystem/drive/DriveSubsystem.h"

class TeleopDrive : public frc2::CommandHelper<frc2::Command, TeleopDrive> {
public:
  TeleopDrive(DriveSubsystem *driveSubsystem, std::function<double()> xStrafe,
              std::function<double()> yStrafe, std::function<double()> turn,
              bool enableSlewRate = true);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  DriveSubsystem *m_driveSubsystem;
  std::function<double()> m_xStrafe;
  std::function<double()> m_yStrafe;
  std::function<double()> m_turn;

  bool m_enableSlewRate;

  frc::SlewRateLimiter<units::scalar> m_xLimiter{3.0 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_yLimiter{3.0 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{3.0 / 1_s};

  double ApplyDeadband(double value, double deadband);
};
