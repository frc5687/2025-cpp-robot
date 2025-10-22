
#include "subsystem/drive/DriveSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "frc/DriverStation.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "pathplanner/lib/config/ModuleConfig.h"
#include "pathplanner/lib/config/RobotConfig.h"
#include "subsystem/drive/PoseEstimator.h"
#include "subsystem/drive/SwerveConstants.h"

DriveSubsystem::DriveSubsystem(std::unique_ptr<ModuleIO> frontLeft,
                               std::unique_ptr<ModuleIO> frontRight,
                               std::unique_ptr<ModuleIO> backLeft,
                               std::unique_ptr<ModuleIO> backRight,
                               std::unique_ptr<GyroIO> gyro)
    : LoggedSubsystem("DriveSubsystem"),
      m_modules{std::make_unique<Module>(std::move(frontLeft)),
                std::make_unique<Module>(std::move(frontRight)),
                std::make_unique<Module>(std::move(backLeft)),
                std::make_unique<Module>(std::move(backRight))},
      m_gyro(std::move(gyro)) {
  pathplanner::ModuleConfig moduleConfig(
      Constants::SwerveDrive::Module::kWheelRadius,
      Constants::SwerveDrive::Module::kMaxModuleLinearSpeed,
      Constants::SwerveDrive::Module::kFrictionCoefficient,
      Constants::SwerveDrive::Module::kDriveMotor,
      Constants::SwerveDrive::Module::kDriveGearRatio,
      Constants::SwerveDrive::Module::kDriveSupplyCurrentLimit, 1.0);
  m_robotConfig = pathplanner::RobotConfig(
      Constants::SwerveDrive::kMass, Constants::SwerveDrive::kMomentOfInertia,
      moduleConfig,
      std::vector<frc::Translation2d>(
          Constants::SwerveDrive::kModuleTranslations.begin(),
          Constants::SwerveDrive::kModuleTranslations.end()));
  m_gyro->Reset();
  PoseEstimator::Config config{};
  config.odometryXStdDev = 0.08;        // Trust odometry more on smooth field
  config.baseXYStdDev = 0.15;           // Conservative vision trust
  config.odometryThetaStdDev = 0.0001;  // Really trust IMU
  config.baseThetaStdDev = 0.2;         // Dont really trust camera rotations
  config.singleTagPenalty = 5.5;
  m_odometryThread =
      std::make_unique<OdometryThread>(m_modules, m_gyro, config);
  StartOdometryThread();
  pathplanner::AutoBuilder::configure(
      [this]() { return GetPose(); },  // Robot pose supplier
      [this](frc::Pose2d pose) {
        ResetPose(pose);
      },  // Method to reset odometry (will be called if your auto has a
          // starting pose)
      [this]() {
        return GetChassisSpeeds();
      },  // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      [this](auto speeds, auto feedforwards) {
        Drive(speeds);
      },  // Method that will drive the robot given ROBOT RELATIVE
          // ChassisSpeeds. Also optionally outputs individual module
          // feedforwards
      std::make_shared<
          pathplanner::PPHolonomicDriveController>(  // PPHolonomicController is
                                                     // the built in path
                                                     // following controller for
                                                     // holonomic drive trains
          pathplanner::PIDConstants(5.0, 0.0,
                                    0.0),           // Translation PID constants
          pathplanner::PIDConstants(5.0, 0.0, 0.0)  // Rotation PID constants
          ),
      m_robotConfig,  // The robot configuration
      []() {
        // Boolean supplier that controls when the path will be mirrored for the
        // red alliance This will flip the path being followed to the red side
        // of the field. THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance) {
          return alliance.value() == frc::DriverStation::Alliance::kRed;
        }
        return false;
      },
      this  // Reference to this subsystem to set requirements
  );
}

DriveSubsystem::~DriveSubsystem() {
  StopOdometryThread();
}

void DriveSubsystem::StartOdometryThread() {
  if (m_odometryThread) {
    m_odometryThread->Start();
    Log("OdometryThreadStarted", true);
  }
}

void DriveSubsystem::StopOdometryThread() {
  if (m_odometryThread) {
    m_odometryThread->Stop();
    Log("OdometryThreadStopped", true);
  }
}

void DriveSubsystem::SetOdometryFrequency(units::hertz_t frequency) {
  if (m_odometryThread) {
    m_odometryThread->SetUpdateFrequency(frequency);
  }
}

void DriveSubsystem::Drive(const frc::ChassisSpeeds& speeds) {
  frc::ChassisSpeeds discretizedSpeeds =
      frc::ChassisSpeeds::Discretize(speeds, 20_ms);
  std::array<frc::SwerveModuleState, 4> moduleStates =
      m_kinematics.ToSwerveModuleStates(discretizedSpeeds);
  SetModuleStates(moduleStates);
}

void DriveSubsystem::DriveFieldRelative(const frc::ChassisSpeeds& speeds) {
  // Get the current heading from threaded odometry
  auto robotRelative =
      frc::ChassisSpeeds::FromFieldRelativeSpeeds(speeds, GetHeading());
  Drive(robotRelative);
}

void DriveSubsystem::SetModuleStates(
    const std::array<frc::SwerveModuleState,
                     Constants::SwerveDrive::kModuleCount>& states) {
  wpi::array<frc::SwerveModuleState, Constants::SwerveDrive::kModuleCount>
      desaturatedStates = states;
  frc::SwerveDriveKinematics<Constants::SwerveDrive::kModuleCount>::
      DesaturateWheelSpeeds(&desaturatedStates, m_maxLinearSpeed);

  for (size_t i = 0; i < Constants::SwerveDrive::kModuleCount; i++) {
    m_modules[i]->SetDesiredState(desaturatedStates[i]);
    m_desiredStates[i] = desaturatedStates[i];
  }
}

void DriveSubsystem::Stop() {
  for (auto& module : m_modules) {
    module->Stop();
  }
}

void DriveSubsystem::LockWheels() {
  constexpr std::array<frc::SwerveModuleState,
                       Constants::SwerveDrive::kModuleCount>
      lockStates{frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}},
                 frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}},
                 frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}},
                 frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}}};
  SetModuleStates(lockStates);
}

frc::Pose2d DriveSubsystem::GetPose() const {
  if (m_odometryThread) {
    return m_odometryThread->GetOdometryPose();
  }
  return frc::Pose2d{};
}

frc::Rotation2d DriveSubsystem::GetHeading() const {
  if (m_odometryThread) {
    auto data = m_odometryThread->GetLatestData();
    return data.gyroAngle;
  }
  return frc::Rotation2d{};
}

void DriveSubsystem::ResetHeading(units::degree_t heading) {
  m_gyro->Reset(heading);

  // Update pose with new heading
  // auto currentPose = GetPose();
  // ResetPose(frc::Pose2d{currentPose.Translation(),
  // frc::Rotation2d{heading}});
}

frc::ChassisSpeeds DriveSubsystem::GetChassisSpeeds() const {
  if (m_odometryThread) {
    return m_odometryThread->GetChassisSpeeds();
  }
  return frc::ChassisSpeeds{};
}

frc::ChassisSpeeds DriveSubsystem::GetFieldRelativeSpeeds() const {
  auto robotSpeeds = GetChassisSpeeds();
  return frc::ChassisSpeeds::FromRobotRelativeSpeeds(robotSpeeds, GetHeading());
}

std::array<frc::SwerveModuleState, Constants::SwerveDrive::kModuleCount>
DriveSubsystem::GetModuleStates() const {
  if (m_odometryThread) {
    return m_odometryThread->GetModuleStates();
  }

  // Fallback to direct module access
  return GetModuleData(&Module::GetState);
}

std::array<frc::SwerveModulePosition, Constants::SwerveDrive::kModuleCount>
DriveSubsystem::GetModulePositions() const {
  if (m_odometryThread) {
    return m_odometryThread->GetModulePositions();
  }

  // Fallback to direct module access
  return GetModuleData(&Module::GetPosition);
}

template <typename T>
std::array<T, Constants::SwerveDrive::kModuleCount>
DriveSubsystem::GetModuleData(T (Module::*getter)() const) const {
  std::array<T, Constants::SwerveDrive::kModuleCount> data;
  for (size_t i = 0; i < Constants::SwerveDrive::kModuleCount; i++) {
    data[i] = (m_modules[i].get()->*getter)();
  }
  return data;
}

// Statistics methods
units::second_t DriveSubsystem::GetOdometryLoopTime() const {
  if (m_odometryThread) {
    return m_odometryThread->GetAverageLoopTime();
  }
  return 0_s;
}

size_t DriveSubsystem::GetOdometrySuccessRate() const {
  if (m_odometryThread) {
    auto successful = m_odometryThread->GetSuccessfulBatches();
    auto failed = m_odometryThread->GetFailedBatches();
    if (successful + failed > 0) {
      return (successful * 100) / (successful + failed);
    }
  }
  return 0;
}
void DriveSubsystem::ResetPose(const frc::Pose2d& pose) {
  if (m_odometryThread) {
    m_odometryThread->ResetPose(pose);
  }
}

// Configuration methods
void DriveSubsystem::SetMaxSpeeds(units::meters_per_second_t linear,
                                  units::radians_per_second_t angular) {
  m_maxLinearSpeed = std::min(linear, Constants::SwerveDrive::kMaxLinearSpeed);
  m_maxAngularSpeed =
      std::min(angular, Constants::SwerveDrive::kMaxAngularSpeed);
}

void DriveSubsystem::SetBrakeMode(bool brake) {
  for (auto& module : m_modules) {
    module->SetBrakeMode(brake);
  }
}

void DriveSubsystem::ConfigureClosedLoop() {
  for (auto& module : m_modules) {
    module->ConfigureClosedLoop();
  }
}

std::array<bool, Constants::SwerveDrive::kModuleCount>
DriveSubsystem::GetModuleConnectionStatus() const {
  std::array<bool, Constants::SwerveDrive::kModuleCount> status;
  for (size_t i = 0; i < Constants::SwerveDrive::kModuleCount; i++) {
    status[i] = m_modules[i]->IsConnected();
  }
  return status;
}

bool DriveSubsystem::IsAtPose(const frc::Pose2d& pose,
                              units::meter_t tolerance) const {
  auto currentPose = GetPose();
  auto distance = currentPose.Translation().Distance(pose.Translation());
  return distance < tolerance;
}

// This is handled by the odometry thread
void DriveSubsystem::UpdateInputs() {}

void DriveSubsystem::LogTelemetry() {
  if (m_odometryThread) {
    auto data = m_odometryThread->GetLatestData();
    if (data.isValid) {
      Log("Pose", data.pose);
      Log("Odometry Pose", data.pose);
      Log("EsimatedPose", m_odometryThread->GetEstimatedPose());
      Log("ModuleStates", data.moduleStates);
      Log("Speeds", data.chassisSpeeds);
      Log("FieldSpeeds", GetFieldRelativeSpeeds());
      Log("Gyro/Yaw", data.gyroAngle);
    }
  }

  Log("DesiredModuleStates", m_desiredStates);
  Log("OdometryLoopTime", GetOdometryLoopTime().value() * 1000);
  Log("OdometrySuccessRate", static_cast<double>(GetOdometrySuccessRate()));

  auto connectionStatus = GetModuleConnectionStatus();
  bool allConnected =
      std::all_of(connectionStatus.begin(), connectionStatus.end(),
                  [](bool connected) { return connected; });
  Log("AllModulesConnected", allConnected);
}
