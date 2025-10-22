
#include "subsystem/vision/VisionSubsystem.h"

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "subsystem/vision/VisionConstants.h"

VisionSubsystem::VisionSubsystem(std::unique_ptr<VisionIO> io,
                                 std::shared_ptr<OdometryThread> odometryThread)
    : LoggedSubsystem("Vision"),
      m_io(std::move(io)),
      m_odometryThread(odometryThread) {}

void VisionSubsystem::UpdateInputs() {
  // This is really only for Sim
  SetRobotPose(m_odometryThread->GetOdometryPose());

  m_io->UpdateInputs(m_inputs);
  for (auto measurement : m_inputs.visionPoseMeasurements) {
    m_odometryThread->AddVisionMeasurement(measurement.second);
  }
}

void VisionSubsystem::LogTelemetry() {
  for (auto est : m_inputs.visionPoseMeasurements) {
    std::string key = "Estimated Pose" + est.first;
    Log(key, est.second.pose);
  }

  std::vector<frc::Pose3d> tags;
  for (auto tag : m_inputs.cameraTagObservations) {
    std::string cam = tag.first;

    auto it = Constants::Vision::kTransformMap.find(cam);
    if (it == Constants::Vision::kTransformMap.end()) {
      std::cerr << "Failed to find camera: " << cam << "\n";
      return;
    }
    frc::Transform3d cameraTransform = it->second;
    frc::Transform3d camTransformToTarget = tag.second.Transform();

    frc::Pose3d robotPose3d{m_odometryThread->GetEstimatedPose()};
    frc::Pose3d tagPoseInField = robotPose3d.TransformBy(cameraTransform)
                                     .TransformBy(camTransformToTarget);

    tags.push_back(tagPoseInField);
  }

  Log("Tag Poses", tags);
}
