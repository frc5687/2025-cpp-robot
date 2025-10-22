
#pragma once

#include <frc/geometry/Transform3d.h>
#include <photon/targeting/PhotonTrackedTarget.h>
#include <units/angle.h>
#include <units/time.h>

#include <algorithm>

class AprilTagObservation final {
 public:
  AprilTagObservation(int id, const frc::Transform3d& transform,
                      double ambiguity, double area, units::degree_t yaw,
                      units::degree_t pitch, units::second_t timestamp,
                      double confidence)
      : m_id{id},
        m_transform{transform},
        m_ambiguity{ambiguity},
        m_area{area},
        m_yaw{yaw},
        m_pitch{pitch},
        m_timestamp{timestamp},
        m_confidence{confidence} {}

  static AprilTagObservation FromPhotonVision(
      const photon::PhotonTrackedTarget& target, units::second_t timestamp) {
    const int id = target.GetFiducialId();
    const frc::Transform3d transform = target.GetBestCameraToTarget();
    const double ambiguity = target.GetPoseAmbiguity();
    const double area = target.GetArea();
    const units::degree_t yaw{target.GetYaw()};
    const units::degree_t pitch{target.GetPitch()};
    const double confidence = std::clamp(1.0 - ambiguity, 0.0, 1.0);
    return AprilTagObservation(id, transform, ambiguity, area, yaw, pitch,
                               timestamp, confidence);
  }

  [[nodiscard]]
  int Id() const {
    return m_id;
  }
  [[nodiscard]]
  const frc::Transform3d& Transform() const {
    return m_transform;
  }
  [[nodiscard]]
  double Ambiguity() const {
    return m_ambiguity;
  }
  [[nodiscard]]
  double Area() const {
    return m_area;
  }
  [[nodiscard]]
  units::degree_t Yaw() const {
    return m_yaw;
  }
  [[nodiscard]]
  units::degree_t Pitch() const {
    return m_pitch;
  }
  [[nodiscard]]
  units::second_t Timestamp() const {
    return m_timestamp;
  }
  [[nodiscard]]
  double Confidence() const {
    return m_confidence;
  }

 private:
  const int m_id;
  const frc::Transform3d m_transform;
  const double m_ambiguity;
  const double m_area;
  const units::degree_t m_yaw;
  const units::degree_t m_pitch;
  const units::second_t m_timestamp;
  const double m_confidence;
};
