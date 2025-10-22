
#pragma once

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/temperature.h>
#include <units/torque.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "ModuleConfig.h"

struct ModuleIOInputs {
  units::meters_per_second_t driveVelocity{0.0};
  units::meter_t drivePosition{0.0};
  frc::Rotation2d steerAngle{0_rad};
  units::turn_t steerPosition{0.0_tr};
  units::ampere_t driveCurrentDraw{0.0};
  units::newton_meter_t driveTorque{0.0};
  units::volt_t driveAppliedVolts{0.0};
  units::volt_t steerAppliedVolts{0.0};
  units::celsius_t driveTemperature{0.0_degC};
  units::celsius_t steerTemperature{0.0_degC};
  bool driveConnected{false};
  bool steerConnected{false};
  bool encoderConnected{false};
};

class ModuleIO {
 public:
  virtual ~ModuleIO() = default;

  virtual void UpdateInputs(ModuleIOInputs& inputs, bool isBatched = false) = 0;
  virtual void SetDesiredState(const frc::SwerveModuleState& state) = 0;
  virtual ModuleConfig GetModuleConfig() = 0;
  virtual void Stop() = 0;
  virtual void SetBrakeMode(bool brake) {}
  virtual void ResetDriveEncoder() {}
  virtual void ConfigureClosedLoop() {}
};
