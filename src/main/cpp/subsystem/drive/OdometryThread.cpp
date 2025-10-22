
#include "subsystem/drive/OdometryThread.h"

#include <fmt/format.h>
#include <frc/Timer.h>

#include <memory>
#include <mutex>

#include "ctre/phoenix6/StatusSignal.hpp"
#include "subsystem/drive/PigeonIO.h"
#include "subsystem/drive/PoseEstimator.h"
#include "subsystem/drive/SwerveConstants.h"
#include "subsystem/drive/module/CTREModuleIO.h"
#include "utils/Logger.h"

OdometryThread::OdometryThread(
    std::array<std::unique_ptr<Module>, Constants::SwerveDrive::kModuleCount>&
        modules,
    std::unique_ptr<GyroIO>& gyro, const PoseEstimator::Config& estimatorConfig)
    : m_modules(modules),
      m_gyro(gyro),
      m_odometry(m_kinematics, frc::Rotation2d{}, GetModulePositions(),
                 frc::Pose2d{}),
      m_estimator(std::make_unique<PoseEstimator>(estimatorConfig)) {
  m_loopTimes.fill(0_s);
  SetupBatchedSignals();
  m_notifier = std::make_unique<frc::Notifier>([this] { PeriodicUpdate(); });
  m_notifier->SetName("OdometryThread");
  m_startTime = frc::Timer::GetFPGATimestamp();

  Logger::Instance().Log("OdometryThread/Initialized", true);
  Logger::Instance().Log("OdometryThread/AllModulesCTRE", m_allModulesAreCTRE);
}

OdometryThread::~OdometryThread() {
  Stop();
}

void OdometryThread::Start() {
  if (m_isRunning.exchange(true)) {
    return;
  }

  m_notifier->StartPeriodic(m_updatePeriod.load());
  Logger::Instance().Log("OdometryThread/UpdateFrequency",
                         m_updateFrequency.load().value());
  Logger::Instance().Log("OdometryThread/UpdatePeriod_ms",
                         m_updatePeriod.load().value() * 1000);
}

void OdometryThread::Stop() {
  if (!m_isRunning.exchange(false)) {
    return;
  }

  Logger::Instance().Log("OdometryThread/Stopping", true);

  if (m_notifier) {
    m_notifier->Stop();
  }

  auto totalRunTime = frc::Timer::GetFPGATimestamp() - m_startTime;
  auto expectedIterations = totalRunTime * m_updateFrequency.load();
  auto actualIterations = m_totalIterations.load();

  Logger::Instance().Log("OdometryThread/TotalRunTime_s", totalRunTime.value());
  Logger::Instance().Log("OdometryThread/ExpectedIterations",
                         expectedIterations.value());
  Logger::Instance().Log("OdometryThread/ActualIterations",
                         static_cast<double>(actualIterations));

  if (expectedIterations.value() > 0) {
    auto accuracyPercent =
        (static_cast<double>(actualIterations) / expectedIterations.value()) *
        100.0;
    Logger::Instance().Log("OdometryThread/TimingAccuracy_%", accuracyPercent);
  }

  Logger::Instance().Log("OdometryThread/Stopped", true);
}

void OdometryThread::PeriodicUpdate() {
  if (!m_isRunning.load()) {
    return;
  }

  auto startTime = frc::Timer::GetFPGATimestamp();
  m_totalIterations++;

  bool inputsValid = false;
  if (m_allModulesAreCTRE) {
    inputsValid = UpdateBatchedInputs();
  } else {
    for (auto& mod : m_modules) {
      mod->Periodic();
    }
    m_gyro->UpdateInputs(m_gyroInputs, false);
    inputsValid = true;
  }

  if (inputsValid) {
    UpdateOdometry();
    UpdatePoseEstimator();
  }

  auto endTime = frc::Timer::GetFPGATimestamp();
  auto loopTime = endTime - startTime;
  UpdateStatistics(loopTime);

  m_lastIterationTime.store(startTime);

  auto updatePeriod = m_updatePeriod.load();
  if (loopTime > updatePeriod * 0.8) {
    static auto lastWarning = 0_s;
    if (startTime - lastWarning > 1_s) {
      Logger::Instance().Log("OdometryThread/ExcessiveLoopTime_ms",
                             units::millisecond_t{loopTime}.value());
      Logger::Instance().Log("OdometryThread/TargetPeriod_ms",
                             units::millisecond_t{updatePeriod}.value());
      lastWarning = startTime;
    }
  }
}

bool OdometryThread::SetupBatchedSignals() {
  size_t writeIdx = 0;

  for (auto& mod : m_modules) {
    if (!mod) {
      m_allModulesAreCTRE = false;
      return false;
    }

    ModuleIO& io = mod->GetModuleIO();
    auto* ctre = dynamic_cast<CTREModuleIO*>(&io);
    if (!ctre) {
      m_allModulesAreCTRE = false;
      return false;
    }

    auto signals = ctre->GetOdometrySignals();
    mod->SetIsBatchedSignals(true);

    for (auto* signal : signals) {
      if (writeIdx >= Constants::SwerveDrive::Odometry::kTotalSignals) {
        m_allModulesAreCTRE = false;
        return false;
      }
      m_batchedSignals[writeIdx++] = signal;
    }
  }

  auto* pigeon = dynamic_cast<PigeonIO*>(m_gyro.get());
  auto imuSignals = pigeon->GetBatchedSignals();
  for (auto* signal : imuSignals) {
    if (writeIdx >= Constants::SwerveDrive::Odometry::kTotalSignals) {
      m_allModulesAreCTRE = false;
      return false;
    }
    m_batchedSignals[writeIdx++] = signal;
  }

  if (writeIdx != Constants::SwerveDrive::Odometry::kTotalSignals) {
    m_allModulesAreCTRE = false;
    return false;
  }

  m_allModulesAreCTRE = true;
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      m_updateFrequency.load(), m_batchedSignals);
  return true;
}

bool OdometryThread::UpdateBatchedInputs() {
  if (!m_allModulesAreCTRE) {
    return false;
  }

  // auto status = ctre::phoenix6::BaseStatusSignal::WaitForAll(m_batchTimeout,
  // m_batchedSignals);
  auto status = ctre::phoenix6::BaseStatusSignal::RefreshAll(m_batchedSignals);

  if (status == ctre::phoenix::StatusCode::OK) {
    m_successfulBatches++;

    for (auto& mod : m_modules) {
      mod->Periodic();
    }

    m_gyro->UpdateInputs(m_gyroInputs, true);

    return true;
  } else {
    m_failedBatches++;

    static auto lastLogTime = 0_s;
    auto currentTime = frc::Timer::GetFPGATimestamp();
    if (currentTime - lastLogTime > 1_s) {
      Logger::Instance().Log("OdometryThread/BatchFailure",
                             static_cast<int>(status));
      Logger::Instance().Log("OdometryThread/BatchTimeout_ms",
                             m_batchTimeout.value());
      lastLogTime = currentTime;
    }

    return false;
  }
}

void OdometryThread::UpdateOdometry() {
  auto currentTime = frc::Timer::GetFPGATimestamp();

  auto lastTimestamp = m_lastValidTimestamp.load();
  if (lastTimestamp != 0_s) {
    auto timestampDelta = currentTime - lastTimestamp;
    if (timestampDelta > kMaxTimestampDelta) {
      Logger::Instance().Log("OdometryThread/LargeTimestampDelta",
                             timestampDelta.value());
    }
  }
  m_lastValidTimestamp.store(currentTime);

  std::array<frc::SwerveModulePosition, Constants::SwerveDrive::kModuleCount>
      modulePositions;
  std::array<frc::SwerveModuleState, Constants::SwerveDrive::kModuleCount>
      moduleStates;

  for (size_t i = 0; i < Constants::SwerveDrive::kModuleCount; i++) {
    modulePositions[i] = m_modules[i]->GetPosition();
    moduleStates[i] = m_modules[i]->GetState();
  }

  auto chassisSpeeds = m_kinematics.ToChassisSpeeds(moduleStates);
  m_gyro->UpdateWithOdometry(chassisSpeeds);

  {
    std::scoped_lock lock(m_dataMutex);
    auto newPose = m_odometry.Update(m_gyroInputs.yaw, modulePositions);

    m_latestData.pose = newPose;
    m_latestData.modulePositions = modulePositions;
    m_latestData.moduleStates = moduleStates;
    m_latestData.chassisSpeeds = chassisSpeeds;
    m_latestData.gyroAngle = m_gyroInputs.yaw;
    m_latestData.timestamp = currentTime;
    m_latestData.isValid = true;
  }
}

void OdometryThread::UpdatePoseEstimator() {
  std::scoped_lock lock(m_dataMutex);
  m_estimator->UpdatePoseEstimate(m_latestData);
}

void OdometryThread::UpdateStatistics(units::second_t loopTime) {
  {
    std::scoped_lock lock(m_statisticsMutex);

    m_loopTimes[m_loopTimeIndex] = loopTime;
    m_loopTimeIndex = (m_loopTimeIndex + 1) % kStatisticsWindowSize;

    units::second_t total = 0_s;
    for (const auto& time : m_loopTimes) {
      total += time;
    }
    m_averageLoopTime.store(total / kStatisticsWindowSize);

    auto currentMax = m_maxLoopTime.load();
    while (loopTime > currentMax &&
           !m_maxLoopTime.compare_exchange_weak(currentMax, loopTime)) {
    }
  }

  static auto lastStatsLog = 0_s;
  auto currentTime = frc::Timer::GetFPGATimestamp();
  if (currentTime - lastStatsLog > 5_s) {
    Logger::Instance().Log("OdometryThread/AvgLoopTime_ms",
                           m_averageLoopTime.load().value() * 1000);
    Logger::Instance().Log("OdometryThread/MaxLoopTime_ms",
                           m_maxLoopTime.load().value() * 1000);
    Logger::Instance().Log("OdometryThread/SuccessfulBatches",
                           static_cast<double>(m_successfulBatches.load()));
    Logger::Instance().Log("OdometryThread/FailedBatches",
                           static_cast<double>(m_failedBatches.load()));
    Logger::Instance().Log("OdometryThread/TotalIterations",
                           static_cast<double>(m_totalIterations.load()));

    auto totalBatches = m_successfulBatches.load() + m_failedBatches.load();
    if (totalBatches > 0) {
      double successRate = static_cast<double>(m_successfulBatches.load()) /
                           totalBatches * 100.0;
      Logger::Instance().Log("OdometryThread/SuccessRate_%", successRate);
    }

    auto runTime = currentTime - m_startTime;
    if (runTime > 0_s) {
      auto actualFreq =
          static_cast<double>(m_totalIterations.load()) / runTime.value();
      Logger::Instance().Log("OdometryThread/ActualFrequency_Hz", actualFreq);
      Logger::Instance().Log("OdometryThread/TargetFrequency_Hz",
                             m_updateFrequency.load().value());

      auto freqError = std::abs(actualFreq - m_updateFrequency.load().value());
      Logger::Instance().Log("OdometryThread/FrequencyError_Hz", freqError);
    }

    lastStatsLog = currentTime;
  }
}
void OdometryThread::AddVisionMeasurement(
    const VisionMeasurement& measurement) {
  std::scoped_lock lock(m_dataMutex);
  m_estimator->AddVisionMeasurement(measurement);
}

void OdometryThread::AddVisionMeasurement(const frc::Pose3d& pose3d,
                                          units::second_t timestamp,
                                          int tagCount, double avgDistance,
                                          double confidence, double ambiguity) {
  std::scoped_lock lock(m_dataMutex);
  m_estimator->AddVisionMeasurement(pose3d, timestamp, tagCount, avgDistance,
                                    confidence, ambiguity);
}

void OdometryThread::SetVisionEnabled(bool enabled) {
  std::scoped_lock lock(m_dataMutex);
  m_estimator->SetVisionEnabled(enabled);
}

bool OdometryThread::IsVisionEnabled() const {
  std::scoped_lock lock(m_dataMutex);
  return m_estimator->IsVisionEnabled();
}

OdometryData OdometryThread::GetLatestData() const {
  std::scoped_lock lock(m_dataMutex);
  return m_latestData;
}

frc::Pose2d OdometryThread::GetOdometryPose() const {
  std::scoped_lock lock(m_dataMutex);
  return m_latestData.pose;
}

frc::Pose2d OdometryThread::GetEstimatedPose() const {
  std::scoped_lock lock(m_dataMutex);
  return m_estimator->GetEstimatedPose();
}

std::array<frc::SwerveModulePosition, Constants::SwerveDrive::kModuleCount>
OdometryThread::GetModulePositions() const {
  std::scoped_lock lock(m_dataMutex);
  return m_latestData.modulePositions;
}

std::array<frc::SwerveModuleState, Constants::SwerveDrive::kModuleCount>
OdometryThread::GetModuleStates() const {
  std::scoped_lock lock(m_dataMutex);
  return m_latestData.moduleStates;
}

frc::ChassisSpeeds OdometryThread::GetChassisSpeeds() const {
  std::scoped_lock lock(m_dataMutex);
  return m_latestData.chassisSpeeds;
}

void OdometryThread::ResetPose(const frc::Pose2d& pose) {
  std::array<frc::SwerveModulePosition, Constants::SwerveDrive::kModuleCount>
      currentPositions;
  for (size_t i = 0; i < Constants::SwerveDrive::kModuleCount; i++) {
    currentPositions[i] = m_modules[i]->GetPosition();
  }

  std::scoped_lock lock(m_dataMutex);
  m_odometry.ResetPosition(m_gyroInputs.yaw, currentPositions, pose);
  m_latestData.pose = pose;
  m_latestData.modulePositions = currentPositions;
  m_estimator->ResetPose(pose);

  Logger::Instance().Log("OdometryThread/PoseReset", pose);
}

void OdometryThread::ResetPoseKeepRotation(const frc::Pose2d& pose) {
  std::scoped_lock lock(m_dataMutex);
  frc::Rotation2d currentRotation;
  currentRotation = m_latestData.gyroAngle;
  frc::Pose2d newPose{pose.Translation(), currentRotation};
  ResetPose(newPose);
  m_estimator->ResetPose(newPose);
}

void OdometryThread::SetUpdateFrequency(units::hertz_t frequency) {
  bool wasRunning = m_isRunning.load();

  if (wasRunning) {
    Stop();
  }

  m_updateFrequency.store(frequency);
  auto updatePeriod = 1.0 / frequency;
  m_updatePeriod.store(updatePeriod);

  Logger::Instance().Log("OdometryThread/UpdateFrequency", frequency.value());
  Logger::Instance().Log("OdometryThread/UpdatePeriod_ms",
                         updatePeriod.value() * 1000);

  if (wasRunning) {
    Start();
  }
}
