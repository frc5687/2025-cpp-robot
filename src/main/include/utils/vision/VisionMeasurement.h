
#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>

struct VisionMeasurement {
  frc::Pose2d pose{};
  frc::Pose3d pose3d{};
  units::second_t timestamp{0};

  int tagCount{0};
  double avgTagDistance{0.0};
  double confidence{1.0};
  double ambiguity{0.0};

  double xyStdDev{0.1};
  double thetaStdDev{0.05};
};
