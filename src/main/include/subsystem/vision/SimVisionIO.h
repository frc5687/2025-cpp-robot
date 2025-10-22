
#pragma once

#include <photon/simulation/VisionSystemSim.h>

#include <memory>
#include <string>
#include <unordered_map>

#include "VisionIO.h"
#include "subsystem/vision/SimulatedPhotonVisionCamera.h"

class SimVisionIO : public VisionIO {
 public:
  SimVisionIO();

  void SetRobotPose(const frc::Pose2d& pose) override { m_robotPose = pose; }
  void UpdateInputs(VisionIOInputs& inputs) override;

 private:
  std::shared_ptr<photon::VisionSystemSim> m_visionSim;
  std::unordered_map<std::string, std::unique_ptr<SimulatedPhotonVisionCamera>>
      m_cams;

  frc::Pose2d m_robotPose{};  // updated externally each loop
};
