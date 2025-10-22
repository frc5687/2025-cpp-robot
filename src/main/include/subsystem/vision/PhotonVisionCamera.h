
// subsystem/vision/PhotonVisionCamera.h
#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform3d.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <photon/targeting/PhotonTrackedTarget.h>
#include <units/time.h>

#include <algorithm>
#include <array>
#include <optional>
#include <string>
#include <vector>

#include "Constants.h"
#include "subsystem/vision/Camera.h"
#include "utils/vision/AprilTagObservation.h"

class PhotonVisionCamera : public Camera {
 public:
  PhotonVisionCamera(const std::string& name,
                     const frc::Transform3d& robotToCamera)
      : m_name{name},
        m_camera{name},
        m_estimator{Constants::Field::kFieldTagLayout,
                    photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
                    robotToCamera},
        m_robotToCamera{robotToCamera} {}

  VisionResult GetLatestResult() override {
    VisionResult out{};

    std::vector<photon::PhotonPipelineResult> results =
        m_camera.GetAllUnreadResults();
    if (results.empty()) {
      return out;
    }

    const photon::PhotonPipelineResult& latestResult = results.back();
    const units::second_t frameTs = latestResult.GetTimestamp();

    if (latestResult.HasTargets()) {
      for (const auto& t : latestResult.GetTargets()) {
        out.tags.emplace_back(
            AprilTagObservation::FromPhotonVision(t, frameTs));
      }
    }

    if (auto est = m_estimator.Update(latestResult)) {
      // est->estimatedPose.ToPose2d();
      int tagCount = est->targetsUsed.size();
      VisionMeasurement measurement;
      measurement.timestamp = est->timestamp;
      measurement.tagCount = tagCount;
      if (tagCount == 1) {
        double ambiguity = est->targetsUsed[0].GetPoseAmbiguity();
        measurement.ambiguity = ambiguity;
        measurement.confidence = std::clamp(1.0 - ambiguity, 0.0, 1.0);
      } else {
        // multiple tags do not need ambiguity and conf
        measurement.ambiguity = -1.0;
        measurement.confidence = -1.0;
      }
      measurement.pose3d = est->estimatedPose;
      measurement.pose = est->estimatedPose.ToPose2d();
      measurement.xyStdDev = 0.3;     // TODO: clean
      measurement.thetaStdDev = 0.9;  // TODO: Clean
      out.poseEstimate.emplace(measurement);
    }

    return out;
  }

 protected:
  const std::string m_name;
  photon::PhotonCamera m_camera;
  photon::PhotonPoseEstimator m_estimator;
  const frc::Transform3d m_robotToCamera;
};
