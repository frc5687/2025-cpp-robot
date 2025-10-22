
#include "subsystem/drive/module/SimModuleIO.h"

#include <random>

#include "subsystem/drive/SwerveConstants.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/velocity.h"
#include "units/voltage.h"
#include "utils/Utils.h"

using namespace Constants::SwerveDrive::Module;

SimModuleIO::SimModuleIO(const ModuleConfig& config)
    : m_config(config),
      m_driveSim(frc::LinearSystemId::DCMotorSystem(kDriveMotor, kDriveInertia,
                                                    kDriveGearRatio),
                 kDriveMotor),
      m_steerSim(frc::LinearSystemId::DCMotorSystem(kSteerMotor, kSteerInertia,
                                                    kSteerGearRatio),
                 kSteerMotor),
      m_drivePid(PID::DriveVelocity::kSimP, PID::DriveVelocity::kSimI,
                 PID::DriveVelocity::kSimD),
      m_steerPid(PID::SteerPosition::kSimP, PID::SteerPosition::kSimI,
                 PID::SteerPosition::kSimD),
      m_driveFF(PID::DriveVelocity::kSimS, PID::DriveVelocity::kSimV,
                PID::DriveVelocity::kSimA),
      m_steerFF(PID::SteerPosition::kSimS, PID::SteerPosition::kSimV,
                PID::SteerPosition::kSimA) {
  {
    static std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<double> uni(-kPi, kPi);
    m_steerAngle = units::radian_t{uni(rng)};
  }

  m_steerSim.SetState(m_steerAngle, 0_rad_per_s);
  m_steerPid.EnableContinuousInput(-kPi, kPi);
}

void SimModuleIO::UpdateInputs(ModuleIOInputs& inputs, bool IsBatched) {
  const auto batteryVoltage =
      units::volt_t{frc::RobotController::GetBatteryVoltage()};

  m_driveSim.SetInputVoltage(
      std::clamp(m_driveAppliedVolts, -batteryVoltage, batteryVoltage));
  m_steerSim.SetInputVoltage(
      std::clamp(m_steerAppliedVolts, -batteryVoltage, batteryVoltage));

  constexpr auto kDt = 4_ms;
  m_driveSim.Update(kDt);
  m_steerSim.Update(kDt);

  const units::radians_per_second_t wheel_omega =
      m_driveSim.GetAngularVelocity();
  const units::meters_per_second_t wheel_mps =
      WheelOmegaToMps(wheel_omega, kWheelRadius);

  m_drivePosition += wheel_mps * kDt;

  m_steerAngle = WrapToPi(m_steerSim.GetAngularPosition());

  inputs.driveVelocity = wheel_mps;
  inputs.drivePosition = m_drivePosition;
  inputs.steerAngle = frc::Rotation2d{m_steerAngle};
  inputs.steerPosition = m_steerAngle;
  inputs.driveAppliedVolts = m_driveAppliedVolts;
  inputs.steerAppliedVolts = m_steerAppliedVolts;
  inputs.driveCurrentDraw =
      units::ampere_t{std::abs(m_driveSim.GetCurrentDraw().value())};
  inputs.driveTorque = m_driveSim.GetTorque();
  inputs.driveTemperature = 25_degC;
  inputs.steerTemperature = 25_degC;
  inputs.driveConnected = true;
  inputs.steerConnected = true;
  inputs.encoderConnected = true;
}

void SimModuleIO::SetDesiredState(const frc::SwerveModuleState& state) {
  frc::SwerveModuleState optimized = state;
  optimized.Optimize(frc::Rotation2d{m_steerAngle});

  const units::meters_per_second_t current_wheel_mps =
      WheelOmegaToMps(m_driveSim.GetAngularVelocity(), kWheelRadius);
  const units::meters_per_second_t target_wheel_mps = optimized.speed;

  const units::volt_t drivePIDOutput{m_drivePid.Calculate(
      current_wheel_mps.value(), target_wheel_mps.value())};

  const units::volt_t driveFFOutput = m_driveFF.Calculate(target_wheel_mps);
  m_driveAppliedVolts = drivePIDOutput + driveFFOutput;

  const units::radian_t current_angle =
      WrapToPi(m_steerSim.GetAngularPosition());
  const units::radian_t target_angle = WrapToPi(optimized.angle.Radians());

  const units::volt_t steerPIDOutput{
      m_steerPid.Calculate(current_angle.value(), target_angle.value())};

  m_steerAppliedVolts = steerPIDOutput;
}

ModuleConfig SimModuleIO::GetModuleConfig() {
  return m_config;
}

void SimModuleIO::Stop() {
  m_driveAppliedVolts = 0_V;
  m_steerAppliedVolts = 0_V;
}
