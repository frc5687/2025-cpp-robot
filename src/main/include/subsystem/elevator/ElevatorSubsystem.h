
#pragma once

#include <memory>

#include "subsystem/LoggedSubsystem.h"
#include "subsystem/elevator/ElevatorIO.h"

class ElevatorSubsystem : public LoggedSubsystem {
public:
  explicit ElevatorSubsystem(std::unique_ptr<ElevatorIO> io);
  ~ElevatorSubsystem() = default;
  void SetElevatorHeight(units::meter_t desiredHeight);
  void SetVoltage(units::volt_t);
  bool AtSetpoint();

protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

private:
  std::unique_ptr<ElevatorIO> m_io;
  ElevatorIOInputs m_inputs{};
  units::meter_t m_desiredPosition;
};
