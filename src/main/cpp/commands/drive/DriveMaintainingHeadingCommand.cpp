
#include "commands/drive/DriveMaintainingHeadingCommand.h"

#include <frc/MathUtil.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <cmath>
#include <numbers>

#include "Constants.h"
#include "subsystem/drive/SwerveConstants.h"

DriveMaintainingHeadingCommand::DriveMaintainingHeadingCommand(
    DriveSubsystem *driveSubsystem, std::function<double()> throttle,
    std::function<double()> strafe, std::function<double()> turn,
    bool enableSlewRate)
    : m_driveSubsystem(driveSubsystem), m_throttleSupplier(throttle),
      m_strafeSupplier(strafe), m_turnSupplier(turn),
      m_enableSlewRate(enableSlewRate), m_joystickLastTouched(-1.0) {
  AddRequirements(driveSubsystem);
  SetName("Drive Maintaining Heading");

  m_headingController.EnableContinuousInput(-std::numbers::pi,
                                            std::numbers::pi);
  m_headingController.SetTolerance(0.05); // 0.05 radians (~3 degrees) tolerance

  m_headingController.SetIntegratorRange(-1.0, 1.0);
}

void DriveMaintainingHeadingCommand::Initialize() {
  m_headingSetpoint = std::nullopt;
  m_joystickLastTouched = -1.0;
  m_headingController.Reset();

  m_xLimiter.Reset(0.0);
  m_yLimiter.Reset(0.0);
  m_rotLimiter.Reset(0.0);
}

void DriveMaintainingHeadingCommand::Execute() {
  double throttle = m_throttleSupplier();
  double strafe = m_strafeSupplier();
  double turnInput = m_turnSupplier();

  throttle = ApplyDeadband(throttle, Constants::kJoystickDeadband);
  strafe = ApplyDeadband(strafe, Constants::kJoystickDeadband);
  turnInput = ApplyDeadband(turnInput, Constants::kSteerJoystickDeadband);

  if (m_enableSlewRate) {
    throttle = m_xLimiter.Calculate(throttle);
    strafe = m_yLimiter.Calculate(strafe);
    turnInput = m_rotLimiter.Calculate(turnInput);
  }

  auto xVelocity = throttle * Constants::SwerveDrive::kMaxLinearSpeed;
  auto yVelocity = strafe * Constants::SwerveDrive::kMaxLinearSpeed;

  if (IsTurnInputActive(turnInput, Constants::kSteerJoystickDeadband)) {
    m_joystickLastTouched = frc::Timer::GetFPGATimestamp().value();
  }

  bool useManualRotation = false;
  auto currentSpeeds = m_driveSubsystem->GetChassisSpeeds();
  auto currentAngularVel = currentSpeeds.omega;

  if (IsTurnInputActive(turnInput, Constants::kSteerJoystickDeadband) ||
      (frc::Timer::GetFPGATimestamp().value() - m_joystickLastTouched < 0.25 &&
       units::math::abs(currentAngularVel) > 10.0_deg_per_s)) {
    useManualRotation = true;
  }

  units::radians_per_second_t rotVelocity;

  if (useManualRotation) {
    rotVelocity = turnInput * Constants::SwerveDrive::kMaxAngularSpeed;
    m_headingSetpoint = std::nullopt; // Clear heading setpoint
    m_headingController.Reset();      // Reset PID controller
  } else {
    if (!m_headingSetpoint.has_value()) {
      m_headingSetpoint = m_driveSubsystem->GetHeading();
      m_headingController.Reset();
    }

    rotVelocity = CalculateHeadingCorrection(m_driveSubsystem->GetHeading(),
                                             m_headingSetpoint.value());
  }

  m_driveSubsystem->DriveFieldRelative(
      frc::ChassisSpeeds{xVelocity, yVelocity, rotVelocity});
}

void DriveMaintainingHeadingCommand::End(bool interrupted) {
  m_driveSubsystem->Stop();
  m_headingSetpoint = std::nullopt;
  m_headingController.Reset();
}

bool DriveMaintainingHeadingCommand::IsFinished() { return false; }

void DriveMaintainingHeadingCommand::SetHeadingPID(double kP, double kI,
                                                   double kD) {
  m_headingController.SetPID(kP, kI, kD);
}

double DriveMaintainingHeadingCommand::ApplyDeadband(double value,
                                                     double deadband) {
  if (std::abs(value) < deadband) {
    return 0.0;
  }
  return (value - std::copysign(deadband, value)) / (1.0 - deadband);
}

units::radians_per_second_t
DriveMaintainingHeadingCommand::CalculateHeadingCorrection(
    frc::Rotation2d current, frc::Rotation2d target) {
  double output = m_headingController.Calculate(current.Radians().value(),
                                                target.Radians().value());

  output = std::clamp(output, -Constants::SwerveDrive::kMaxAngularSpeed.value(),
                      Constants::SwerveDrive::kMaxAngularSpeed.value());

  return units::radians_per_second_t{output};
}

bool DriveMaintainingHeadingCommand::IsTurnInputActive(double turnInput,
                                                       double deadband) {
  return std::abs(turnInput) > deadband;
}
