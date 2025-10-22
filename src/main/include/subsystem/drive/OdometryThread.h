
#pragma once

#include <frc/Notifier.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <units/time.h>

#include <array>
#include <atomic>
#include <memory>
#include <mutex>

#include <ctre/phoenix6/StatusSignal.hpp>

#include "GyroIO.h"
#include "OdometryData.h"
#include "PoseEstimator.h"
#include "SwerveConstants.h"
#include "module/Module.h"

class OdometryThread {
 public:
  OdometryThread(std::array<std::unique_ptr<Module>,
                            Constants::SwerveDrive::kModuleCount>& modules,
                 std::unique_ptr<GyroIO>& gyro,
                 const PoseEstimator::Config& estimatorConfig);
  ~OdometryThread();

  void Start();
  void Stop();

  OdometryData GetLatestData() const;
  frc::Pose2d GetOdometryPose() const;
  frc::Pose2d GetEstimatedPose() const;
  std::array<frc::SwerveModulePosition, Constants::SwerveDrive::kModuleCount>
  GetModulePositions() const;
  std::array<frc::SwerveModuleState, Constants::SwerveDrive::kModuleCount>
  GetModuleStates() const;
  frc::ChassisSpeeds GetChassisSpeeds() const;

  void ResetPose(const frc::Pose2d& pose);
  void ResetPoseKeepRotation(const frc::Pose2d& pose);
  void SetUpdateFrequency(units::hertz_t frequency);

  units::second_t GetAverageLoopTime() const { return m_averageLoopTime; }
  units::second_t GetMaxLoopTime() const { return m_maxLoopTime; }
  size_t GetSuccessfulBatches() const { return m_successfulBatches; }
  size_t GetFailedBatches() const { return m_failedBatches; }
  size_t GetTotalIterations() const { return m_totalIterations; }

  void AddVisionMeasurement(const VisionMeasurement& measurement);
  void AddVisionMeasurement(const frc::Pose3d& pose3d,
                            units::second_t timestamp, int tagCount = 1,
                            double avgDistance = 2.0, double confidence = 1.0,
                            double ambiguity = 0.0);
  void SetVisionEnabled(bool enabled);
  bool IsVisionEnabled() const;

 private:
  void PeriodicUpdate();
  bool SetupBatchedSignals();
  bool UpdateBatchedInputs();
  void UpdateOdometry();
  void UpdatePoseEstimator();
  void UpdateStatistics(units::second_t loopTime);

  mutable std::mutex m_dataMutex;
  OdometryData m_latestData;
  std::unique_ptr<frc::Notifier> m_notifier;
  std::atomic<bool> m_isRunning{false};

  std::array<std::unique_ptr<Module>, Constants::SwerveDrive::kModuleCount>&
      m_modules;
  std::unique_ptr<GyroIO>& m_gyro;
  GyroIOInputs m_gyroInputs{};

  std::array<ctre::phoenix6::BaseStatusSignal*,
             Constants::SwerveDrive::Odometry::kTotalSignals>
      m_batchedSignals{};
  bool m_allModulesAreCTRE{false};
  units::millisecond_t m_batchTimeout{5_ms};

  frc::SwerveDriveKinematics<Constants::SwerveDrive::kModuleCount> m_kinematics{
      Constants::SwerveDrive::kModuleTranslations};
  frc::SwerveDriveOdometry<Constants::SwerveDrive::kModuleCount> m_odometry;

  std::atomic<units::hertz_t> m_updateFrequency{250_Hz};
  std::atomic<units::second_t> m_updatePeriod{4_ms};

  std::atomic<units::second_t> m_averageLoopTime{0_s};
  std::atomic<units::second_t> m_maxLoopTime{0_s};
  std::atomic<size_t> m_successfulBatches{0};
  std::atomic<size_t> m_failedBatches{0};
  std::atomic<size_t> m_totalIterations{0};

  static constexpr size_t kStatisticsWindowSize = 50;
  std::array<units::second_t, kStatisticsWindowSize> m_loopTimes{};
  size_t m_loopTimeIndex{0};
  std::mutex m_statisticsMutex;

  std::atomic<units::second_t> m_lastValidTimestamp{0_s};
  static constexpr units::second_t kMaxTimestampDelta{0.1_s};

  std::atomic<units::second_t> m_lastIterationTime{0_s};
  units::second_t m_startTime{0_s};

  std::unique_ptr<PoseEstimator> m_estimator;
};
