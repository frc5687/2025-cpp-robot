
#pragma once

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>

#include "GyroIO.h"
#include "SwerveConstants.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/temperature.h"
#include "utils/CANDevice.h"

class PigeonIO : public GyroIO {
 public:
  explicit PigeonIO(CANDevice device);

  void UpdateInputs(GyroIOInputs& inputs, bool isBatched) override;
  void Reset(units::degree_t angle = 0_deg) override;
  const std::array<ctre::phoenix6::BaseStatusSignal*,
                   Constants::SwerveDrive::Odometry::kIMUSignals>
  GetBatchedSignals() const {
    return m_batchedSignals;
  }

 private:
  ctre::phoenix6::hardware::Pigeon2 m_pigeon;
  ctre::phoenix6::StatusSignal<units::degree_t> m_yawSignal;
  ctre::phoenix6::StatusSignal<units::degrees_per_second_t> m_yawRateSignal;
  ctre::phoenix6::StatusSignal<units::degree_t> m_pitchSignal;
  ctre::phoenix6::StatusSignal<units::degree_t> m_rollSignal;
  ctre::phoenix6::StatusSignal<units::celsius_t> m_temp;

  std::array<ctre::phoenix6::BaseStatusSignal*,
             Constants::SwerveDrive::Odometry::kIMUSignals>
      m_batchedSignals;
  units::radians_per_second_t m_lastYawRate{0};
  units::second_t m_lastTime{0};
};
