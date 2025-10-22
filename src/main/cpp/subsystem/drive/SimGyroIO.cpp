
#include "subsystem/drive/SimGyroIO.h"

void SimGyroIO::UpdateInputs(GyroIOInputs& inputs, bool isBatched) {
  const auto currentTime = units::second_t{frc::Timer::GetFPGATimestamp()};
  if (m_lastUpdateTime != 0_s) {
    if (const auto dt = currentTime - m_lastUpdateTime; dt < 100_ms) {
      m_angle = m_angle + frc::Rotation2d{m_rate * dt};
    }
  }
  m_lastUpdateTime = currentTime;

  inputs.yaw = m_angle;
  inputs.yawRate = m_rate;
  inputs.connected = true;
  inputs.temperature = 25_degC;

  if (m_lastTime != 0_s) {
    auto dt = currentTime - m_lastTime;
    inputs.yawAcceleration = (m_rate - m_lastRate) / dt;
  }
  m_lastRate = m_rate;
  m_lastTime = currentTime;

  inputs.timestamp = currentTime.value();
}

void SimGyroIO::Reset(units::degree_t angle) {
  m_angle = frc::Rotation2d{angle};
  m_rate = 0_rad_per_s;
  m_lastRate = 0_rad_per_s;
}

void SimGyroIO::UpdateWithOdometry(const frc::ChassisSpeeds& robotSpeeds) {
  // Update the simulated gyro rate based on robot movement, I didn't have a
  // nice way to do this (or not yet).
  m_rate = robotSpeeds.omega;
}
