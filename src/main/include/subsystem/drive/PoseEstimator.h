
#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/interpolation/TimeInterpolatableBuffer.h>
#include <units/time.h>
#include <units/velocity.h>

#include <atomic>
#include <vector>

#include "OdometryData.h"
#include "utils/vision/VisionMeasurement.h"

class PoseEstimator {
 public:
  static constexpr units::second_t kPoseBufferTime{1.5_s};

  struct Config {
    double odometryXStdDev{0.1};
    double odometryYStdDev{0.1};
    double odometryThetaStdDev{0.05};

    double baseXYStdDev{0.1};
    double baseThetaStdDev{0.05};
    double singleTagPenalty{2.0};
    double distancePenaltyRate{0.3};
    double confidenceThreshold{0.8};
    double confidencePenaltyRate{3.0};

    double maxVisionDistance{6.0};
    double maxTimestampAge{0.25};
    double minConfidence{0.4};
    double maxMovementSpeed{8.0};
  };

  explicit PoseEstimator(const Config& config);
  ~PoseEstimator() = default;
  void UpdatePoseEstimate(const OdometryData& latestOdometry);
  void ResetPose(const frc::Pose2d& pose);
  void ResetPoseKeepRotation(const frc::Pose2d& pose);

  void AddVisionMeasurement(const VisionMeasurement& measurement);
  void AddVisionMeasurement(const frc::Pose3d& pose3d,
                            units::second_t timestamp, int tagCount = 1,
                            double avgDistance = 2.0, double confidence = 1.0,
                            double ambiguity = 0.0);

  frc::Pose2d GetEstimatedPose() const;
  units::meters_per_second_t GetCurrentVelocity() const;

  void SetVisionEnabled(bool enabled) { m_visionEnabled = enabled; }
  bool IsVisionEnabled() const { return m_visionEnabled; }

  size_t GetProcessedMeasurements() const { return m_processedCount; }
  size_t GetRejectedMeasurements() const { return m_rejectedCount; }
  units::second_t GetLastVisionTime() const { return m_lastVisionTime; }
  const Config& GetConfig() const { return m_config; }

 private:
  void ProcessPendingMeasurements(const frc::Pose2d& currentOdometryPose);
  void ProcessVisionMeasurement(const frc::Pose2d& currentOdometryPose,
                                const VisionMeasurement& measurement);

  bool ValidateMeasurement(VisionMeasurement& measurement);
  void CalculateStandardDeviations(VisionMeasurement& measurement);
  bool IsReasonablePose(const frc::Pose2d& pose) const;
  bool IsReasonableMovement(const frc::Pose2d& from, const frc::Pose2d& to,
                            units::second_t deltaTime) const;

  std::array<double, 3> CalculateKalmanGain(
      const VisionMeasurement& measurement) const;
  frc::Pose2d ApplyVisionUpdate(const frc::Pose2d& estimate,
                                const VisionMeasurement& measurement,
                                const std::array<double, 3>& gain) const;

  const Config m_config;

  frc::Pose2d m_estimatedPose{};
  frc::Pose2d m_lastOdometryPose{};
  units::meters_per_second_t m_currentVelocity{0_mps};
  units::second_t m_lastUpdateTime{0_s};

  std::vector<VisionMeasurement> m_pendingMeasurements;
  frc::TimeInterpolatableBuffer<frc::Pose2d> m_poseBuffer{kPoseBufferTime};

  bool m_visionEnabled{true};

  size_t m_processedCount{0};
  size_t m_rejectedCount{0};
  std::atomic<units::second_t> m_lastVisionTime{0_s};

  static constexpr double kMinX = -1.0;
  static constexpr double kMaxX = 17.0;
  static constexpr double kMinY = -1.0;
  static constexpr double kMaxY = 9.0;
};
