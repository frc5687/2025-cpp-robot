
#pragma once

#include <string>
#include <unordered_map>

#include "frc/geometry/Pose2d.h"
#include "utils/vision/AprilTagObservation.h"
#include "utils/vision/VisionMeasurement.h"

struct VisionIOInputs {
  std::unordered_map<std::string, AprilTagObservation> cameraTagObservations{};
  std::unordered_map<std::string, VisionMeasurement> visionPoseMeasurements{};
};

class VisionIO {
 public:
  virtual ~VisionIO() = default;
  virtual void UpdateInputs(VisionIOInputs& inputs) = 0;
  virtual void SetRobotPose(const frc::Pose2d& pose) = 0;
};
