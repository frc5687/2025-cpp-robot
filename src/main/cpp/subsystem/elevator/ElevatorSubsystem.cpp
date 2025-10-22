
#include "subsystem/elevator/ElevatorSubsystem.h"

#include <units/math.h>

#include <utility>

#include "subsystem/elevator/ElevatorConstants.h"
#include "subsystem/elevator/ElevatorIO.h"
#include "subsystem/LoggedSubsystem.h"

ElevatorSubsystem::ElevatorSubsystem(std::unique_ptr<ElevatorIO> io)
    : LoggedSubsystem("Elevator"), m_io(std::move(io)) {}

void ElevatorSubsystem::UpdateInputs() {
  m_io->UpdateInputs(m_inputs);
}

void ElevatorSubsystem::SetElevatorHeight(units::meter_t desiredHeight) {
  m_desiredPosition = desiredHeight;
  m_io->SetElevatorHeight(m_desiredPosition);
}

void ElevatorSubsystem::SetVoltage(units::volt_t volts) {
  m_io->SetElevatorVoltage(volts);
}

bool ElevatorSubsystem::AtSetpoint() {
  return units::math::abs(m_desiredPosition - m_inputs.elevatorPosition) <
         Constants::Elevator::kPositionTolerance;
}

void ElevatorSubsystem::LogTelemetry() {
  Log("Elevator Desired Height", m_desiredPosition.value());
  Log("At Setpoint", AtSetpoint());
  Log("Elevator Height", m_inputs.elevatorPosition.value());
  Log("Elevator Velocity", m_inputs.elevatorVelocity.value());
  Log("Elevator Left Position", m_inputs.leftMotorPosition.value());
  Log("Elevator Right Position", m_inputs.rightMotorPosition.value());
}
