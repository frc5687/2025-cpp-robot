
#pragma once

#include <frc/simulation/ElevatorSim.h>

#include "ElevatorIO.h"
#include "frc/controller/ProfiledPIDController.h"

class SimElevatorIO : public ElevatorIO {
 public:
  SimElevatorIO();
  ~SimElevatorIO() = default;

  void UpdateInputs(ElevatorIOInputs& inputs) override;
  void SetElevatorHeight(units::meter_t desiredHeight) override;

 private:
  frc::sim::ElevatorSim m_elevatorSim;
  frc::ProfiledPIDController<units::meter> m_pidController;
};
