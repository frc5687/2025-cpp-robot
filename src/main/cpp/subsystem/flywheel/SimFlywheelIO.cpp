#include "subsystem/flywheel/SimFlywheelIO.h"
#include <frc/system/plant/LinearSystemId.h>
#include <frc/Timer.h>
#include "subsystem/flywheel/FlywheelConstants.h"

SimFlywheelIO::SimFlywheelIO() : m_flywheelSim(
    frc::LinearSystemId::FlywheelSystem(
        Constants::Flywheel::kMotor,
        Constants::Flywheel::kFlywheelMOI,
        Constants::Flywheel::kGearRatio
    ),
    Constants::Flywheel::kMotor,
    {}) {
}

void SimFlywheelIO::UpdateInputs(FlywheelIOInputs& inputs) {
    m_flywheelSim.Update(20_ms);
    inputs.flywheelVelockty = m_flywheelSim.GetAngularVelocity();
    inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void SimFlywheelIO::SetFlywheelVelocity(units::revolutions_per_minute_t desiredRPM) {
    auto volts = 0_V;
    m_flywheelSim.SetInputVoltage(volts);
};

