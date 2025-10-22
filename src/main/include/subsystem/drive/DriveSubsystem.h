
#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <units/acceleration.h>
#include <units/temperature.h>

#include <array>
#include <memory>
#include <vector>

#include "GyroIO.h"
#include "OdometryThread.h"
#include "SwerveConstants.h"
#include "module/Module.h"
#include "subsystem/LoggedSubsystem.h"

class DriveSubsystem : public LoggedSubsystem {
 public:
  DriveSubsystem(std::unique_ptr<ModuleIO> frontLeft,
                 std::unique_ptr<ModuleIO> frontRight,
                 std::unique_ptr<ModuleIO> backLeft,
                 std::unique_ptr<ModuleIO> backRight,
                 std::unique_ptr<GyroIO> gyro);

  ~DriveSubsystem();

  void Drive(const frc::ChassisSpeeds& speeds);
  void DriveFieldRelative(const frc::ChassisSpeeds& speeds);
  void SetModuleStates(
      const std::array<frc::SwerveModuleState,
                       Constants::SwerveDrive::kModuleCount>& states);

  void Stop();
  void LockWheels();

  frc::Pose2d GetPose() const;
  frc::Rotation2d GetHeading() const;
  void ResetHeading(units::degree_t heading = 0_deg);

  // State information
  frc::ChassisSpeeds GetChassisSpeeds() const;
  frc::ChassisSpeeds GetFieldRelativeSpeeds() const;
  std::array<frc::SwerveModuleState, Constants::SwerveDrive::kModuleCount>
  GetModuleStates() const;
  std::array<frc::SwerveModulePosition, Constants::SwerveDrive::kModuleCount>
  GetModulePositions() const;

  // Threaded odometry control
  void StartOdometryThread();
  void StopOdometryThread();
  void SetOdometryFrequency(units::hertz_t frequency);
  void ResetPose(const frc::Pose2d& pose);

  // Odometry stats
  units::second_t GetOdometryLoopTime() const;
  size_t GetOdometrySuccessRate() const;

 public:
  void SetMaxSpeeds(units::meters_per_second_t linear,
                    units::radians_per_second_t angular);
  void SetBrakeMode(bool brake);
  void ConfigureClosedLoop();
  bool IsAtPose(const frc::Pose2d& pose,
                units::meter_t tolerance = 0.1_m) const;
  std::array<bool, Constants::SwerveDrive::kModuleCount>
  GetModuleConnectionStatus() const;
  std::shared_ptr<OdometryThread> GetOdometryThread() {
    return m_odometryThread;
  }
  std::shared_ptr<const OdometryThread> GetOdometryThread() const {
    return m_odometryThread;
  }

 protected:
  void UpdateInputs() override;
  void LogTelemetry() override;

 private:
  template <typename T>
  std::array<T, Constants::SwerveDrive::kModuleCount> GetModuleData(
      T (Module::*getter)() const) const;

  std::vector<std::unique_ptr<Module>> m_test;
  std::array<std::unique_ptr<Module>, Constants::SwerveDrive::kModuleCount>
      m_modules;
  std::unique_ptr<GyroIO> m_gyro;
  std::shared_ptr<OdometryThread> m_odometryThread;

  frc::SwerveDriveKinematics<Constants::SwerveDrive::kModuleCount> m_kinematics{
      Constants::SwerveDrive::kModuleTranslations};

  std::array<frc::SwerveModuleState, Constants::SwerveDrive::kModuleCount>
      m_desiredStates;
  units::meters_per_second_t m_maxLinearSpeed =
      Constants::SwerveDrive::kMaxLinearSpeed;
  units::radians_per_second_t m_maxAngularSpeed =
      Constants::SwerveDrive::kMaxAngularSpeed;
  pathplanner::RobotConfig m_robotConfig{};
};
