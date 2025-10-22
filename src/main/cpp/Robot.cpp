
#include "Robot.h"

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

Robot::Robot() {
  frc::DriverStation::SilenceJoystickConnectionWarning(true);
}

void Robot::RobotPeriodic() {
  auto startTime = frc::Timer::GetFPGATimestamp();

  frc2::CommandScheduler::GetInstance().Run();

  auto endTime = frc::Timer::GetFPGATimestamp();
  auto updateTime = (endTime - startTime);
  Logger::Instance().Log("Performance/UpdateTime", updateTime.value() * 1000);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}
void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();
  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}
void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}
void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
