
#include "subsystem/drive/module/Module.h"

#include <fmt/format.h>

#include <string>
#include <utility>

#include "subsystem/drive/SwerveConstants.h"
#include "utils/Logger.h"

Module::Module(std::unique_ptr<ModuleIO> io)
    : m_io(std::move(io)),
      m_position(m_io->GetModuleConfig().modulePosition),
      m_name(ModulePositionToString(m_position)) {}

void Module::Periodic() {
  m_io->UpdateInputs(m_inputs, m_isSignalsBatched);
  LogState();
}

void Module::SetDesiredState(const frc::SwerveModuleState& state) {
  SetDesiredState(state, true);
}

void Module::SetDesiredState(const frc::SwerveModuleState& state,
                             bool optimize) {
  m_desiredState = state;

  // if (optimize && ShouldOptimize(state)) {
  m_optimizedState = state;
  m_optimizedState.Optimize(m_inputs.steerAngle);
  m_optimizedState.CosineScale(m_inputs.steerAngle);
  //} else {
  //  m_optimizedState = state;
  //}

  m_io->SetDesiredState(m_optimizedState);
}

void Module::Stop() {
  m_io->Stop();
  m_desiredState = frc::SwerveModuleState{0_mps, m_inputs.steerAngle};
  m_optimizedState = m_desiredState;
}

frc::SwerveModuleState Module::GetState() const {
  return frc::SwerveModuleState{
      units::meters_per_second_t{
          m_velocityFilter.Calculate(m_inputs.driveVelocity.value())},
      m_inputs.steerAngle};
}

frc::SwerveModulePosition Module::GetPosition() const {
  return frc::SwerveModulePosition{m_inputs.drivePosition, m_inputs.steerAngle};
}

void Module::ResetDrivePosition() const {
  m_io->ResetDriveEncoder();
}

void Module::SetBrakeMode(bool brake) {
  m_io->SetBrakeMode(brake);
}

void Module::ConfigureClosedLoop() {
  m_io->ConfigureClosedLoop();
}

units::ampere_t Module::GetCurrentDraw() const {
  return units::ampere_t{
      m_currentFilter.Calculate(m_inputs.driveCurrentDraw.value())};
}

bool Module::IsConnected() const {
  return m_inputs.driveConnected && m_inputs.steerConnected &&
         m_inputs.encoderConnected;
}

bool Module::ShouldOptimize(const frc::SwerveModuleState& desired) const {
  return units::math::abs(desired.speed) > kOptimizationThreshold;
}

void Module::LogState() {
  const std::string prefix = fmt::format("Module/{}/", m_name);
  auto& logger = Logger::Instance();

  logger.Log(prefix + "Velocity", m_inputs.driveVelocity.value());
  logger.Log(prefix + "Position", m_inputs.drivePosition.value());
  logger.Log(prefix + "Angle", m_inputs.steerAngle);
  logger.Log(prefix + "SteerTurns", m_inputs.steerPosition.value());
  logger.Log(prefix + "CurrentState", GetState());
  logger.Log(prefix + "DesiredState", m_desiredState);
  logger.Log(prefix + "OptimizedState", m_optimizedState);
  logger.Log(prefix + "DriveCurrent", GetCurrentDraw().value());
  logger.Log(prefix + "DriveTorque", GetDriveTorque().value());
  logger.Log(prefix + "DriveForce",
             (GetDriveTorque() / Constants::SwerveDrive::Module::kWheelRadius)
                 .value());
  logger.Log(prefix + "DriveVoltage", m_inputs.driveAppliedVolts.value());
  logger.Log(prefix + "SteerVoltage", m_inputs.steerAppliedVolts.value());
  logger.Log(prefix + "DriveTemp", m_inputs.driveTemperature.value());
  logger.Log(prefix + "SteerTemp", m_inputs.steerTemperature.value());
  logger.Log(prefix + "Connected", IsConnected());

  const auto velocityError = m_desiredState.speed - GetState().speed;
  const auto angleError =
      (m_optimizedState.angle - m_inputs.steerAngle).Radians();
  logger.Log(prefix + "VelocityError", velocityError.value());
  logger.Log(prefix + "AngleError", angleError.value());
}
