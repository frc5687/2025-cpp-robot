
#pragma once

#include <frc/RobotController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/length.h>

#include "ModuleConfig.h"
#include "ModuleIO.h"

class SimModuleIO : public ModuleIO {
 public:
  explicit SimModuleIO(const ModuleConfig& config);

  void UpdateInputs(ModuleIOInputs& inputs, bool isBatched) override;
  void SetDesiredState(const frc::SwerveModuleState& state) override;
  ModuleConfig GetModuleConfig() override;
  void Stop() override;

 private:
  ModuleConfig m_config;

  frc::sim::DCMotorSim m_driveSim;
  frc::sim::DCMotorSim m_steerSim;

  units::volt_t m_driveAppliedVolts{0};
  units::volt_t m_steerAppliedVolts{0};
  units::meter_t m_drivePosition{0};
  units::radian_t m_steerAngle{0};

  frc::PIDController m_drivePid;
  frc::PIDController m_steerPid;
  frc::SimpleMotorFeedforward<units::meters> m_driveFF;
  frc::SimpleMotorFeedforward<units::radians> m_steerFF;
};
