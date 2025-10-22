
#include "subsystem/drive/PigeonIO.h"

#include <frc/RobotController.h>
#include <frc/Timer.h>

#include "ctre/phoenix6/StatusSignal.hpp"
#include "utils/CANDevice.h"

PigeonIO::PigeonIO(CANDevice device)
    : m_pigeon(device.id, device.bus),
      m_yawSignal(m_pigeon.GetYaw()),
      m_yawRateSignal(m_pigeon.GetAngularVelocityZWorld()),
      m_pitchSignal(m_pigeon.GetPitch()),
      m_rollSignal(m_pigeon.GetRoll()),
      m_temp(m_pigeon.GetTemperature()),
      m_batchedSignals{&m_yawSignal, &m_yawRateSignal} {
  ctre::phoenix6::configs::Pigeon2Configuration config{};
  config.MountPose.MountPoseYaw = 0_deg;
  config.MountPose.MountPosePitch = 0_deg;
  config.MountPose.MountPoseRoll = 0_deg;
  m_pigeon.GetConfigurator().Apply(config);
}

void PigeonIO::UpdateInputs(GyroIOInputs& inputs, bool isBatched) {
  if (!isBatched) {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(m_yawSignal, m_yawRateSignal);
  }
  ctre::phoenix6::BaseStatusSignal::RefreshAll(m_pitchSignal, m_rollSignal,
                                               m_temp);

  inputs.yaw = frc::Rotation2d{m_yawSignal.GetValue()};
  inputs.yawRate = m_yawRateSignal.GetValue();
  inputs.pitch = frc::Rotation2d{m_pitchSignal.GetValue()};
  inputs.roll = frc::Rotation2d{m_rollSignal.GetValue()};
  inputs.temperature = m_temp.GetValue();
  inputs.connected = m_yawSignal.GetStatus().IsOK();

  auto currentTime = units::second_t{frc::Timer::GetFPGATimestamp()};
  if (m_lastTime != 0_s) {
    auto dt = currentTime - m_lastTime;
    inputs.yawAcceleration = (inputs.yawRate - m_lastYawRate) / dt;
  }
  m_lastYawRate = inputs.yawRate;
  m_lastTime = currentTime;

  inputs.timestamp = currentTime.value();
}

void PigeonIO::Reset(units::degree_t angle) {
  m_pigeon.SetYaw(angle);
}
