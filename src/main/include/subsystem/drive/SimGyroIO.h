
#pragma once

#include <frc/Timer.h>

#include "GyroIO.h"

class SimGyroIO : public GyroIO {
 public:
  SimGyroIO() = default;

  void UpdateInputs(GyroIOInputs& inputs, bool isBatched) override;
  void Reset(units::degree_t angle = 0_deg) override;

  // Override to update based on robot movement
  void UpdateWithOdometry(const frc::ChassisSpeeds& robotSpeeds) override;

 private:
  frc::Rotation2d m_angle{0_rad};
  units::radians_per_second_t m_rate{0};
  units::radians_per_second_t m_lastRate{0};
  units::second_t m_lastTime{0};
  units::second_t m_lastUpdateTime{0};
};
