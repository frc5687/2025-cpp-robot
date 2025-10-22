
#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

struct ElevatorIOInputs {
  units::turn_t leftMotorPosition{0_tr};
  units::turn_t rightMotorPosition{0_tr};
  units::turns_per_second_t leftMotorVelocity{0_tps};
  units::turns_per_second_t rightMotorVelocity{0_tps};

  units::meter_t elevatorPosition{0_m};                // Average of motors
  units::meters_per_second_t elevatorVelocity{0_mps};  // Average of velocity

  units::second_t timestamp{0_s};
};

class ElevatorIO {
 public:
  virtual ~ElevatorIO() = default;
  virtual void UpdateInputs(ElevatorIOInputs& inputs) = 0;
  virtual void SetElevatorHeight(units::meter_t desiredHeight) = 0;
  virtual void SetElevatorVoltage(units::volt_t voltage) {}
};
