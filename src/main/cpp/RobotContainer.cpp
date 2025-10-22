
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>

#include <memory>
#include <utility>

#include "Constants.h"
#include "HardwareMap.h"
#include "subsystem/drive/PigeonIO.h"
#include "subsystem/drive/SimGyroIO.h"
#include "subsystem/drive/SwerveConstants.h"
#include "subsystem/drive/module/CTREModuleIO.h"
#include "subsystem/drive/module/ModuleConfig.h"
#include "subsystem/drive/module/SimModuleIO.h"
#include "subsystem/elevator/CTREElevatorIO.h"
#include "subsystem/elevator/SimElevatorIO.h"
#include "subsystem/vision/SimVisionIO.h"

RobotContainer::RobotContainer() {
  // Module encoder offsets (tune these per robot)
  constexpr std::array<units::turn_t, 4> kEncoderOffsets{
      0.079569_tr,                // FL
      0.43359375_tr - 0.5_tr,     // FR
      0.35595703125_tr - 0.5_tr,  // BL
      -0.2431540625_tr + 0.5_tr   // BR
  };

  // bool simulation = true;
  if (frc::RobotBase::IsSimulation()) {
    // if (simulation) {
    auto FL = ModuleConfig(ModulePosition::FrontLeft, 0_tr);
    auto FR = ModuleConfig(ModulePosition::FrontRight, 0_tr);
    auto BL = ModuleConfig(ModulePosition::BackLeft, 0_tr);
    auto BR = ModuleConfig(ModulePosition::BackRight, 0_tr);
    m_drive = std::make_unique<DriveSubsystem>(
        std::make_unique<SimModuleIO>(FL), std::make_unique<SimModuleIO>(FR),
        std::make_unique<SimModuleIO>(BL), std::make_unique<SimModuleIO>(BR),
        std::make_unique<SimGyroIO>());
  } else {
    auto FL = std::make_unique<CTREModuleIO>(
        CTREModuleIO::DeviceIDs{HardwareMap::CAN::TalonFX::FrontLeftDrive,
                                HardwareMap::CAN::TalonFX::FrontLeftSteer,
                                HardwareMap::CAN::CANCoder::FrontLeftEncoder},
        ModuleConfig{ModulePosition::FrontLeft, kEncoderOffsets[0]});

    auto FR = std::make_unique<CTREModuleIO>(
        CTREModuleIO::DeviceIDs{HardwareMap::CAN::TalonFX::FrontRightDrive,
                                HardwareMap::CAN::TalonFX::FrontRightSteer,
                                HardwareMap::CAN::CANCoder::FrontRightEncoder},
        ModuleConfig{ModulePosition::FrontRight, kEncoderOffsets[1]});

    auto BL = std::make_unique<CTREModuleIO>(
        CTREModuleIO::DeviceIDs{HardwareMap::CAN::TalonFX::BackLeftDrive,
                                HardwareMap::CAN::TalonFX::BackLeftSteer,
                                HardwareMap::CAN::CANCoder::BackLeftEncoder},
        ModuleConfig{ModulePosition::BackLeft, kEncoderOffsets[2]});

    auto BR = std::make_unique<CTREModuleIO>(
        CTREModuleIO::DeviceIDs{HardwareMap::CAN::TalonFX::BackRightDrive,
                                HardwareMap::CAN::TalonFX::BackRightSteer,
                                HardwareMap::CAN::CANCoder::BackRightEncoder},
        ModuleConfig{ModulePosition::BackRight, kEncoderOffsets[3]});
    auto gyro = std::make_unique<PigeonIO>(HardwareMap::CAN::Pidgeon2::IMU);

    m_drive = std::make_unique<DriveSubsystem>(std::move(FL), std::move(FR),
                                               std::move(BL), std::move(BR),
                                               std::move(gyro));
  }

  if (frc::RobotBase::IsSimulation()) {
    auto elevatorIO = std::make_unique<SimElevatorIO>();
    m_elevator = std::make_unique<ElevatorSubsystem>(std::move(elevatorIO));
  } else {
    auto elevatorIO = std::make_unique<CTREElevatorIO>(
        HardwareMap::CAN::TalonFX::LeftElevator,
        HardwareMap::CAN::TalonFX::RightElevator);
    m_elevator = std::make_unique<ElevatorSubsystem>(std::move(elevatorIO));
  }

  auto visionIO = std::make_unique<SimVisionIO>();
  m_vision = std::make_unique<VisionSubsystem>(std::move(visionIO),
                                               m_drive->GetOdometryThread());
  ConfigureBindings();
}

double RobotContainer::ApplyDeadband(double value, double deadband) {
  if (std::abs(value) < deadband) {
    return 0.0;
  }
  return (value - std::copysign(deadband, value)) / (1.0 - deadband);
}

void RobotContainer::ConfigureBindings() {
  using frc2::cmd::Run;
  using frc2::cmd::RunOnce;

  m_drive->SetDefaultCommand(Run(
      [this] {
        const double xInput =
            ApplyDeadband(-m_driver.GetLeftY(), Constants::kJoystickDeadband);
        const double yInput =
            ApplyDeadband(-m_driver.GetLeftX(), Constants::kJoystickDeadband);
        const double rotInput =
            ApplyDeadband(-m_driver.GetRightX(), Constants::kJoystickDeadband);

        const auto xVelocity = xInput * Constants::SwerveDrive::kMaxLinearSpeed;
        const auto yVelocity = yInput * Constants::SwerveDrive::kMaxLinearSpeed;
        const auto rotVelocity =
            rotInput * Constants::SwerveDrive::kMaxAngularSpeed;

        m_drive->DriveFieldRelative(
            frc::ChassisSpeeds{xVelocity, yVelocity, rotVelocity});
      },
      {m_drive.get()}));

  m_driver.Square().OnTrue(Run([this] { m_elevator->SetElevatorHeight(0.3_m); },
                               {m_elevator.get()}));

  m_driver.Triangle().OnTrue(Run(
      [this] { m_elevator->SetElevatorHeight(0.0_m); }, {m_elevator.get()}));

  m_driver.Cross().OnTrue(Run([this] { m_elevator->SetElevatorHeight(0.1_m); },
                              {m_elevator.get()}));

  m_driver.Circle().OnTrue(Run([this] { m_elevator->SetElevatorHeight(0.6_m); },
                               {m_elevator.get()}));

  // m_driver.Circle().OnTrue(
  //   Run([this] { m_elevator->SetElevatorHeight(0.6_m); },
  //   {m_elevator.get()}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
