
#pragma once

#include <memory>

#include "frc/geometry/Pose2d.h"
#include "subsystem/drive/OdometryThread.h"
#include "subsystem/vision/VisionIO.h"
#include "subsystem/LoggedSubsystem.h"

class VisionSubsystem : public LoggedSubsystem {
 public:
  VisionSubsystem(std::unique_ptr<VisionIO> io,
                  std::shared_ptr<OdometryThread> odometryThread);
  ~VisionSubsystem() = default;

  void SetRobotPose(const frc::Pose2d& pose) { m_io->SetRobotPose(pose); }

 protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

 private:
  std::unique_ptr<VisionIO> m_io;
  std::shared_ptr<OdometryThread> m_odometryThread;
  VisionIOInputs m_inputs;
};
