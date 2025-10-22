
#pragma once

#include <networktables/NetworkTable.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <ctre/phoenix6/sim/CANcoderSimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>

#include "ModuleConfig.h"
#include "ModuleIO.h"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "subsystem/drive/SwerveConstants.h"
#include "utils/CANDevice.h"

class CTREModuleIO : public ModuleIO {
 public:
  struct DeviceIDs {
    CANDevice drive;
    CANDevice steer;
    CANDevice encoder;
  };

  CTREModuleIO(const DeviceIDs& ids, const ModuleConfig& config);

  void UpdateInputs(ModuleIOInputs& inputs, bool isBatched) override;
  void SetDesiredState(const frc::SwerveModuleState& state) override;
  void Stop() override;
  void SetBrakeMode(bool brake) override;
  void ResetDriveEncoder() override;
  void ConfigureClosedLoop() override;
  ModuleConfig GetModuleConfig() override;
  std::array<ctre::phoenix6::BaseStatusSignal*,
             Constants::SwerveDrive::Odometry::kSignalsPerModule>
  GetOdometrySignals() const {
    return m_synchedSignals;
  }

 private:
  ctre::phoenix6::hardware::TalonFX m_driveMotor;
  ctre::phoenix6::hardware::TalonFX m_steerMotor;
  ctre::phoenix6::hardware::CANcoder m_encoder;
  ModuleConfig m_config;

  ctre::phoenix6::StatusSignal<units::turns_per_second_t>&
      m_driveVelocitySignal;
  ctre::phoenix6::StatusSignal<units::turn_t>& m_drivePositionSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_driveCurrentSignal;
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_driveTorqueCurrentSignal;
  ctre::phoenix6::StatusSignal<units::volt_t>& m_driveVoltageSignal;
  ctre::phoenix6::StatusSignal<units::celsius_t>& m_driveTempSignal;
  ctre::phoenix6::StatusSignal<units::turn_t>& m_steerPositionSignal;
  ctre::phoenix6::StatusSignal<units::turns_per_second_t>&
      m_steerVelocitySignal;
  ctre::phoenix6::StatusSignal<units::volt_t>& m_steerVoltageSignal;
  ctre::phoenix6::StatusSignal<units::celsius_t>& m_steerTempSignal;
  ctre::phoenix6::StatusSignal<units::turn_t>& m_encoderPositionSignal;

  std::array<ctre::phoenix6::BaseStatusSignal*,
             Constants::SwerveDrive::Odometry::kSignalsPerModule>
      m_synchedSignals;
  std::array<ctre::phoenix6::BaseStatusSignal*, 6> m_batchedSignals;

  ctre::phoenix6::controls::VelocityVoltage m_driveVelocity{0_tps};
  ctre::phoenix6::controls::PositionVoltage m_steerPosition{0_tr};
  ctre::phoenix6::controls::VoltageOut m_driveVoltage{0_V};

  ctre::phoenix6::configs::TalonFXConfiguration m_driveConfig{};
  ctre::phoenix6::configs::TalonFXConfiguration m_steerConfig{};

  void ConfigureDevices();
  void ConfigureSignalFrequencies();
};
