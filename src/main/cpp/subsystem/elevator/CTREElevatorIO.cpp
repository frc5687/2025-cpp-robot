
#include "subsystem/elevator/CTREElevatorIO.h"

#include "ctre/phoenix6/StatusSignal.hpp"
#include "subsystem/elevator/ElevatorConstants.h"
#include "subsystem/elevator/ElevatorIO.h"
#include "units/velocity.h"
#include "units/voltage.h"

CTREElevatorIO::CTREElevatorIO(const CANDevice& leftMotor,
                               const CANDevice& rightMotor)
    : m_leftMotor(leftMotor.id, leftMotor.bus),
      m_rightMotor(rightMotor.id, rightMotor.bus),
      m_leftMotorPositionSignal(m_leftMotor.GetPosition()),
      m_leftMotorVelocitySignal(m_leftMotor.GetVelocity()),
      m_leftCurrentSignal(m_leftMotor.GetStatorCurrent()),
      m_rightMotorPositionSignal(m_rightMotor.GetPosition()),
      m_rightMotorVelocitySignal(m_rightMotor.GetVelocity()),
      m_rightCurrentSignal(m_rightMotor.GetStatorCurrent()),
      m_batchSignals{&m_leftMotorPositionSignal,  &m_leftMotorVelocitySignal,
                     &m_leftCurrentSignal,        &m_rightMotorPositionSignal,
                     &m_rightMotorVelocitySignal, &m_rightCurrentSignal},
      m_leftController(0_tr),
      m_rightController(0_tr),
      m_leftVoltage(0_V),
      m_rightVoltage(0_V) {
  m_leftConfig.MotorOutput.Inverted =
      Constants::Elevator::kLeftInverted
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  m_rightConfig.MotorOutput.Inverted =
      Constants::Elevator::kRightInverted
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  m_leftConfig.Voltage.PeakForwardVoltage = 12_V;
  m_leftConfig.Voltage.PeakReverseVoltage = -12_V;

  m_rightConfig.Voltage.PeakForwardVoltage = 12_V;
  m_rightConfig.Voltage.PeakReverseVoltage = -12_V;

  m_leftConfig.Slot0.kP = Constants::Elevator::kP;
  m_leftConfig.Slot0.kI = Constants::Elevator::kI;
  m_leftConfig.Slot0.kD = Constants::Elevator::kD;

  m_rightConfig.Slot0.kP = Constants::Elevator::kP;
  m_rightConfig.Slot0.kI = Constants::Elevator::kI;
  m_rightConfig.Slot0.kD = Constants::Elevator::kD;

  m_leftConfig.MotionMagic.MotionMagicCruiseVelocity =
      Constants::Elevator::kMaxVelocity / Constants::Elevator::kDrumRadius *
      Constants::Elevator::kGearRatio * 1_tr;
  m_leftConfig.MotionMagic.MotionMagicAcceleration =
      Constants::Elevator::kMaxAccel / Constants::Elevator::kDrumRadius *
      Constants::Elevator::kGearRatio * 1_tr;
  m_leftConfig.MotionMagic.MotionMagicJerk = 10000_tr_per_s_cu;

  m_rightConfig.MotionMagic.MotionMagicCruiseVelocity =
      Constants::Elevator::kMaxVelocity / Constants::Elevator::kDrumRadius *
      Constants::Elevator::kGearRatio * 1_tr;
  m_rightConfig.MotionMagic.MotionMagicAcceleration =
      Constants::Elevator::kMaxAccel / Constants::Elevator::kDrumRadius *
      Constants::Elevator::kGearRatio * 1_tr;
  m_rightConfig.MotionMagic.MotionMagicJerk = 10000_tr_per_s_cu;

  m_leftMotor.GetConfigurator().Apply(m_leftConfig);
  m_rightMotor.GetConfigurator().Apply(m_rightConfig);
}

void CTREElevatorIO::UpdateInputs(ElevatorIOInputs& inputs) {
  ctre::phoenix6::BaseStatusSignal::RefreshAll(m_batchSignals);

  inputs.leftMotorPosition = m_leftMotorPositionSignal.GetValue();
  inputs.rightMotorPosition = m_rightMotorPositionSignal.GetValue();

  units::meter_t leftPositionMeters = inputs.leftMotorPosition *
                                      Constants::Elevator::kCircumference /
                                      Constants::Elevator::kGearRatio / 1_tr;
  units::meter_t rightPositionMeters = inputs.rightMotorPosition *
                                       Constants::Elevator::kCircumference /
                                       Constants::Elevator::kGearRatio / 1_tr;

  inputs.elevatorPosition = (leftPositionMeters + rightPositionMeters) / 2.0;

  inputs.leftMotorVelocity = m_leftMotorVelocitySignal.GetValue();
  inputs.rightMotorVelocity = m_rightMotorVelocitySignal.GetValue();

  units::meters_per_second_t leftVelocity =
      inputs.leftMotorVelocity * Constants::Elevator::kCircumference /
      Constants::Elevator::kGearRatio / 1_tr;
  units::meters_per_second_t rightVelocity =
      inputs.rightMotorVelocity * Constants::Elevator::kCircumference /
      Constants::Elevator::kGearRatio / 1_tr;

  inputs.elevatorVelocity = (leftVelocity + rightVelocity) / 2.0;

  inputs.timestamp = frc::Timer::GetFPGATimestamp();
}

void CTREElevatorIO::SetElevatorHeight(units::meter_t desiredHeight) {
  units::turn_t leftMotorTurns = desiredHeight /
                                 Constants::Elevator::kCircumference *
                                 Constants::Elevator::kGearRatio * 1_tr;
  units::turn_t rightMotorTurns = desiredHeight /
                                  Constants::Elevator::kCircumference *
                                  Constants::Elevator::kGearRatio * 1_tr;

  m_leftMotor.SetControl(m_leftController.WithPosition(leftMotorTurns));
  m_rightMotor.SetControl(m_rightController.WithPosition(rightMotorTurns));
}

void CTREElevatorIO::SetElevatorVoltage(units::volt_t voltage) {
  m_rightMotor.SetControl(m_rightVoltage.WithOutput(voltage));
  m_leftMotor.SetControl(m_leftVoltage.WithOutput(voltage));
}
