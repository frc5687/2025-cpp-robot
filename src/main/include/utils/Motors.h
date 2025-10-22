
#pragma once

#include <frc/system/plant/DCMotor.h>

static constexpr frc::DCMotor KrakenX44(int numMotors = 1) {
  return frc::DCMotor(12_V, 4.05_Nm, 275_A, 1.4_A, 7530_rpm, numMotors);
}
