
#include "commands/drive/TeleopDrive.h"

#include <frc/kinematics/ChassisSpeeds.h>

#include <cmath>

#include "Constants.h"
#include "subsystem/drive/SwerveConstants.h"

TeleopDrive::TeleopDrive(DriveSubsystem *driveSubsystem,
                         std::function<double()> xStrafe,
                         std::function<double()> yStrafe,
                         std::function<double()> turn, bool enableSlewRate)
    : m_driveSubsystem(driveSubsystem), m_xStrafe(xStrafe), m_yStrafe(yStrafe),
      m_turn(turn), m_enableSlewRate(enableSlewRate) {
  AddRequirements(driveSubsystem);
}

void TeleopDrive::Initialize() {
  m_xLimiter.Reset(0.0);
  m_yLimiter.Reset(0.0);
  m_rotLimiter.Reset(0.0);
}

void TeleopDrive::Execute() {
  double xInput = m_xStrafe();
  double yInput = m_yStrafe();
  double rotInput = m_turn();

  xInput = ApplyDeadband(xInput, Constants::kJoystickDeadband);
  yInput = ApplyDeadband(yInput, Constants::kJoystickDeadband);
  rotInput = ApplyDeadband(rotInput, Constants::kJoystickDeadband);

  if (m_enableSlewRate) {
    xInput = m_xLimiter.Calculate(xInput);
    yInput = m_yLimiter.Calculate(yInput);
    rotInput = m_rotLimiter.Calculate(rotInput);
  }

  auto xVelocity = xInput * Constants::SwerveDrive::kMaxLinearSpeed;
  auto yVelocity = yInput * Constants::SwerveDrive::kMaxLinearSpeed;
  auto rotVelocity = rotInput * Constants::SwerveDrive::kMaxAngularSpeed;

  m_driveSubsystem->DriveFieldRelative(
      frc::ChassisSpeeds{xVelocity, yVelocity, rotVelocity});
}

void TeleopDrive::End(bool interrupted) { m_driveSubsystem->Stop(); }

bool TeleopDrive::IsFinished() { return false; }

double TeleopDrive::ApplyDeadband(double value, double deadband) {
  if (std::abs(value) < deadband) {
    return 0.0;
  }
  return (value - std::copysign(deadband, value)) / (1.0 - deadband);
}
