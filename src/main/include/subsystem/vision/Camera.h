
#pragma once

#include <optional>
#include <vector>

#include "utils/vision/AprilTagObservation.h"
#include "utils/vision/VisionMeasurement.h"

class Camera {
 public:
  struct VisionResult {
    std::vector<AprilTagObservation> tags;
    std::optional<VisionMeasurement> poseEstimate;
  };

  virtual ~Camera() = default;
  virtual VisionResult GetLatestResult() = 0;
};
