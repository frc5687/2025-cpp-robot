
#include "subsystem/elevator/SimElevatorIO.h"

#include "frc/Timer.h"
#include "frc/trajectory/TrapezoidProfile.h"
#include "subsystem/elevator/ElevatorConstants.h"

SimElevatorIO::SimElevatorIO()
    : m_elevatorSim(Constants::Elevator::kMotor,
                    Constants::Elevator::kGearRatio, Constants::Elevator::kMass,
                    Constants::Elevator::kDrumRadius,
                    Constants::Elevator::kMinHeight,
                    Constants::Elevator::kMaxHeight, true, 0_m, {0.001, 0.001}),
      m_pidController(100, 0, 0,
                      frc::TrapezoidProfile<units::meter>::Constraints(
                          Constants::Elevator::kMaxVelocity,
                          Constants::Elevator::kMaxAccel)) {}

void SimElevatorIO::UpdateInputs(ElevatorIOInputs& inputs) {
  m_elevatorSim.Update(20_ms);
  inputs.elevatorPosition = m_elevatorSim.GetPosition();
  inputs.elevatorVelocity = m_elevatorSim.GetVelocity();
  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimElevatorIO::SetElevatorHeight(units::meter_t desiredHeight) {
  auto position = m_elevatorSim.GetPosition();
  auto pidOutput = m_pidController.Calculate(position, desiredHeight);

  m_elevatorSim.SetInputVoltage(units::volt_t{pidOutput});
}
