#include "commands/drive/TeleopDrive.h"

TeleopDrive::TeleopDrive(
        DriveSubsystem* driveSubsystem,
        std::function<double()> xStrafe,
        std::function<double()> yStrafe,
        std::function<double()> turn
        ) :
        m_driveSubsystem(driveSubsystem),
        m_xStrafe(xStrafe),
        m_yStrafe(yStrafe),
        m_turn(turn) {
}

void TeleopDrive::Initialize() {
  // do something
}

bool TeleopDrive::IsFinished() {
  return false;
}