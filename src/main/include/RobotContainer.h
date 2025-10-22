
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandPS5Controller.h>

#include <memory>

#include "subsystem/drive/DriveSubsystem.h"
#include "subsystem/elevator/ElevatorSubsystem.h"
#include "subsystem/vision/VisionSubsystem.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();
  static double ApplyDeadband(double value, double deadband = 0.1);
  std::unique_ptr<DriveSubsystem> m_drive = nullptr;
  std::unique_ptr<ElevatorSubsystem> m_elevator = nullptr;
  std::unique_ptr<VisionSubsystem> m_vision = nullptr;
  frc2::CommandPS5Controller m_driver{0};
};
