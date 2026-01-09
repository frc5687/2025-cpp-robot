#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/voltage.h>
#include <units/time.h>

struct FlywheelIOInputs {
    units::turn_t motorPosition{0_tr};
    units::turns_per_second_t motorVelocity{0_tps};

    units::revolutions_per_minute_t flywheelVelockty{0_rpm};

    units::second_t timestamp{0_s};
};

class FlywheelIO {
public:
    virtual ~FlywheelIO() = default;
    virtual void UpdateInputs(FlywheelIOInputs& inputs) = 0;
    virtual void SetFlywheelVelocity(units::revolutions_per_minute_t desiredRPM) = 0;
    virtual void SetFlywheelVoltage(units::volt_t volts) {}
};
