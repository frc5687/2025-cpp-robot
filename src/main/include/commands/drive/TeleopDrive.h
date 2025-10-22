#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <functional>

#include "subsystem/drive/DriveSubsystem.h"

class TeleopDrive : public frc2::CommandHelper<frc2::Command, TeleopDrive> {
public:
    TeleopDrive(
        DriveSubsystem* driveSubsystem,
        std::function<double()> xStrafe,
        std::function<double()> yStrafe,
        std::function<double()> turn
        );
    void Initialize() override;
    bool IsFinished() override;
private:
    DriveSubsystem* m_driveSubsystem;
    std::function<double()> m_xStrafe;
    std::function<double()> m_yStrafe;
    std::function<double()> m_turn;
};