
#include "subsystem/vision/SimVisionIO.h"

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform3d.h>
#include <units/angle.h>
#include <units/length.h>

#include <algorithm>
#include <map>
#include <memory>
#include <vector>

#include "Constants.h"
#include "subsystem/vision/Camera.h"
#include "subsystem/vision/SimulatedPhotonVisionCamera.h"
#include "subsystem/vision/VisionConstants.h"
#include "utils/vision/AprilTagObservation.h"

using units::degree_t;
using units::meter_t;

inline const AprilTagObservation* BestTag(
    const std::vector<AprilTagObservation>& tags) {
  if (tags.empty())
    return nullptr;
  return &*std::ranges::max_element(
      tags, [](const AprilTagObservation& a, const AprilTagObservation& b) {
        if (a.Confidence() != b.Confidence())
          return a.Confidence() < b.Confidence();
        return a.Area() < b.Area();
      });
}

SimVisionIO::SimVisionIO()
    : m_visionSim(std::make_shared<photon::VisionSystemSim>("MainVision")) {
  m_visionSim->AddAprilTags(Constants::Field::kFieldTagLayout);

  m_cams.emplace(
      "limelightleft",
      std::make_unique<SimulatedPhotonVisionCamera>(
          "limelightleft", Constants::Vision::kRobotToNWCam, m_visionSim));
  m_cams.emplace(
      "limelight-center",
      std::make_unique<SimulatedPhotonVisionCamera>(
          "limelightcenter", Constants::Vision::kRobotToNECam, m_visionSim));
  m_cams.emplace(
      "South_Camera",
      std::make_unique<SimulatedPhotonVisionCamera>(
          "SouthCamera", Constants::Vision::kRobotToSouthCam, m_visionSim));
}

void SimVisionIO::UpdateInputs(VisionIOInputs& inputs) {
  m_visionSim->Update(m_robotPose);

  inputs.cameraTagObservations.clear();
  inputs.visionPoseMeasurements.clear();

  for (auto& [name, cam] : m_cams) {
    Camera::VisionResult res = cam->GetLatestResult();

    if (const AprilTagObservation* best = BestTag(res.tags)) {
      inputs.cameraTagObservations.erase(name);
      inputs.cameraTagObservations.emplace(name, *best);
    } else {
      inputs.cameraTagObservations.erase(name);
    }

    if (res.poseEstimate) {
      inputs.visionPoseMeasurements.erase(name);
      inputs.visionPoseMeasurements.emplace(name, *res.poseEstimate);
    }
  }
}
