
#include "subsystem/drive/PoseEstimator.h"

#include <frc/Timer.h>
#include <frc/geometry/Transform2d.h>
#include <wpi/array.h>

#include <iostream>
#include <vector>

#include "utils/Logger.h"

PoseEstimator::PoseEstimator(const Config& config) : m_config(config) {
  Logger::Instance().Log("PoseEstimator/Initialized", true);
}

void PoseEstimator::UpdatePoseEstimate(const OdometryData& latestOdometry) {
  if (!latestOdometry.isValid) {
    return;
  }

  auto currentTime = latestOdometry.timestamp;

  if (m_lastUpdateTime > 0_s) {
    auto deltaTime = currentTime - m_lastUpdateTime;
    if (deltaTime > 0_s) {
      auto deltaTransform =
          frc::Transform2d{m_lastOdometryPose, latestOdometry.pose};
      auto distance = units::meter_t{
          std::sqrt(deltaTransform.X().value() * deltaTransform.X().value() +
                    deltaTransform.Y().value() * deltaTransform.Y().value())};
      m_currentVelocity = distance / deltaTime;
    }
  }
  frc::Transform2d odometryDelta{m_lastOdometryPose, latestOdometry.pose};
  m_estimatedPose = m_estimatedPose + odometryDelta;
  m_lastOdometryPose = latestOdometry.pose;
  m_lastUpdateTime = currentTime;

  m_poseBuffer.AddSample(currentTime, latestOdometry.pose);
  ProcessPendingMeasurements(latestOdometry.pose);
}

void PoseEstimator::ResetPose(const frc::Pose2d& pose) {
  m_estimatedPose = pose;
  m_lastOdometryPose = pose;
  m_currentVelocity = 0_mps;
  m_poseBuffer.Clear();
  m_poseBuffer.AddSample(frc::Timer::GetFPGATimestamp(), pose);

  Logger::Instance().Log("PoseEstimator/PoseReset", pose);
}

void PoseEstimator::ResetPoseKeepRotation(const frc::Pose2d& pose) {
  frc::Rotation2d currentRotation;
  currentRotation = m_estimatedPose.Rotation();
  frc::Pose2d newPose{pose.Translation(), currentRotation};
  ResetPose(newPose);
}

void PoseEstimator::AddVisionMeasurement(const VisionMeasurement& measurement) {
  if (!m_visionEnabled) {
    return;
  }

  VisionMeasurement validatedMeasurement = measurement;
  if (ValidateMeasurement(validatedMeasurement)) {
    m_pendingMeasurements.push_back(validatedMeasurement);
  } else {
    m_rejectedCount++;
  }
}

void PoseEstimator::AddVisionMeasurement(const frc::Pose3d& pose3d,
                                         units::second_t timestamp,
                                         int tagCount, double avgDistance,
                                         double confidence, double ambiguity) {
  VisionMeasurement measurement;
  measurement.pose3d = pose3d;
  measurement.pose = pose3d.ToPose2d();
  measurement.timestamp = timestamp;
  measurement.tagCount = tagCount;
  measurement.avgTagDistance = avgDistance;
  measurement.confidence = confidence;
  measurement.ambiguity = ambiguity;

  AddVisionMeasurement(measurement);
}

void PoseEstimator::ProcessPendingMeasurements(
    const frc::Pose2d& currentOdometryPose) {
  std::vector<VisionMeasurement> measurements;
  measurements.swap(m_pendingMeasurements);

  for (const auto& measurement : measurements) {
    ProcessVisionMeasurement(currentOdometryPose, measurement);
  }
}

void PoseEstimator::ProcessVisionMeasurement(
    const frc::Pose2d& currentOdometryPose,
    const VisionMeasurement& measurement) {
  auto odometryAtTime = m_poseBuffer.Sample(measurement.timestamp);
  if (!odometryAtTime) {
    Logger::Instance().Log("PoseEstimator/NoInterpolation",
                           measurement.timestamp.value());
    m_rejectedCount++;
    return;
  }

  frc::Transform2d odometryTransform{odometryAtTime.value(),
                                     currentOdometryPose};

  frc::Pose2d estimateAtTime = m_estimatedPose + odometryTransform.Inverse();
  frc::Transform2d measurementDelta{estimateAtTime, measurement.pose};

  auto gain = CalculateKalmanGain(measurement);
  frc::Transform2d scaledCorrection{
      measurementDelta.X() * gain[0], measurementDelta.Y() * gain[1],
      frc::Rotation2d{measurementDelta.Rotation().Radians() * gain[2]}};

  m_estimatedPose = estimateAtTime + scaledCorrection + odometryTransform;

  m_processedCount++;
  m_lastVisionTime = frc::Timer::GetFPGATimestamp();

  static auto lastLog = 0_s;
  auto now = frc::Timer::GetFPGATimestamp();
  if (now - lastLog > 0.2_s) {
    Logger::Instance().Log("PoseEstimator/MeasurementXYStd",
                           measurement.xyStdDev);
    Logger::Instance().Log("PoseEstimator/MeasurementThetaStd",
                           measurement.thetaStdDev);
    Logger::Instance().Log("PoseEstimator/KalmanGain", gain);
    lastLog = now;
  }
}

bool PoseEstimator::ValidateMeasurement(VisionMeasurement& measurement) {
  auto currentTime = frc::Timer::GetFPGATimestamp();

  if (measurement.tagCount <= 0) {
    return false;
  }

  if (currentTime - measurement.timestamp >
      units::second_t{m_config.maxTimestampAge}) {
    // std::cout << "Not valid due to timestamp diff: " << (currentTime -
    // measurement.timestamp).value() << "\n";
    return false;
  }

  if (measurement.confidence > 0 &&
      measurement.confidence < m_config.minConfidence) {
    // std::cout << "Not valid due to confidence: " << measurement.confidence <<
    // "\n";
    return false;
  }

  if (!IsReasonablePose(measurement.pose)) {
    // std::cout << "Not valid due to not a resonable pose.\n";
    return false;
  }

  // NOTE: this works if our pose doesn't drift significantly, otherwise wont
  // converge since filtered out units::second_t deltaTime = 0_s; if
  // (m_lastUpdateTime > 0_s) {
  //   deltaTime = currentTime - measurement.timestamp;
  // }
  // if (deltaTime > 0.01_s &&
  //     !IsReasonableMovement(GetEstimatedPose(), measurement.pose, deltaTime))
  //     {
  //   std::cout << "Not valid due to not a resonable movement.\n";
  //   return false;
  // }

  CalculateStandardDeviations(measurement);

  return true;
}

void PoseEstimator::CalculateStandardDeviations(
    VisionMeasurement& measurement) {
  double xyStd = m_config.baseXYStdDev;
  double thetaStd = m_config.baseThetaStdDev;

  // I'm being lazy with all the magic numbers TODO: Move to constants.
  if (measurement.tagCount <= 1) {
    xyStd *= m_config.singleTagPenalty;
    thetaStd *= m_config.singleTagPenalty * 0.75;
  }

  if (measurement.avgTagDistance > 3.0) {
    double distanceFactor =
        1.0 + (measurement.avgTagDistance - 3.0) * m_config.distancePenaltyRate;
    xyStd *= distanceFactor;
    thetaStd *= distanceFactor * 0.6;
  }

  if (measurement.confidence < m_config.confidenceThreshold) {
    double confidenceFactor =
        1.0 + (m_config.confidenceThreshold - measurement.confidence) *
                  m_config.confidencePenaltyRate;
    xyStd *= confidenceFactor;
    thetaStd *= confidenceFactor;
  }

  if (measurement.ambiguity > 0.2) {
    double ambiguityFactor = 1.0 + measurement.ambiguity * 2.0;
    xyStd *= ambiguityFactor;
    thetaStd *= ambiguityFactor;
  }

  measurement.xyStdDev = std::clamp(xyStd, 0.02, 3.0);
  measurement.thetaStdDev = std::clamp(thetaStd, 0.01, 1.5);
}

std::array<double, 3> PoseEstimator::CalculateKalmanGain(
    const VisionMeasurement& measurement) const {
  std::array<double, 3> gain;
  std::array<double, 3> qVar = {
      m_config.odometryXStdDev * m_config.odometryXStdDev,
      m_config.odometryYStdDev * m_config.odometryYStdDev,
      m_config.odometryThetaStdDev * m_config.odometryThetaStdDev};
  std::array<double, 3> rVar = {
      measurement.xyStdDev * measurement.xyStdDev,
      measurement.xyStdDev * measurement.xyStdDev,
      measurement.thetaStdDev * measurement.thetaStdDev};

  for (int i = 0; i < 3; i++) {
    if (qVar[i] == 0.0 && rVar[i] != 0.0) {
      gain[i] = 0.0;
    } else if (qVar[i] != 0.0 && rVar[i] == 0.0) {
      gain[i] = 1.0;
    } else if (qVar[i] == 0.0 && rVar[i] == 0.0) {
      gain[i] = 0.0;
    } else {
      gain[i] = qVar[i] / (qVar[i] + std::sqrt(qVar[i] * rVar[i]));
    }
  }

  return gain;
}

bool PoseEstimator::IsReasonablePose(const frc::Pose2d& pose) const {
  auto translation = pose.Translation();
  return translation.X().value() >= kMinX && translation.X().value() <= kMaxX &&
         translation.Y().value() >= kMinY && translation.Y().value() <= kMaxY;
}

bool PoseEstimator::IsReasonableMovement(const frc::Pose2d& from,
                                         const frc::Pose2d& to,
                                         units::second_t deltaTime) const {
  if (deltaTime <= 0_s) {
    return true;
  }

  auto delta = to.Translation() - from.Translation();
  auto distance = delta.Norm();
  auto speed = distance / deltaTime;

  return speed < units::meters_per_second_t{m_config.maxMovementSpeed};
}

frc::Pose2d PoseEstimator::GetEstimatedPose() const {
  return m_estimatedPose;
}

units::meters_per_second_t PoseEstimator::GetCurrentVelocity() const {
  return m_currentVelocity;
}
