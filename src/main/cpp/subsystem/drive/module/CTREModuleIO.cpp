
#include "subsystem/drive/module/CTREModuleIO.h"

#include <frc/RobotController.h>

#include "subsystem/drive/SwerveConstants.h"
#include "utils/Utils.h"

using namespace Constants::SwerveDrive::Module;
using namespace ctre::phoenix6;

CTREModuleIO::CTREModuleIO(const DeviceIDs& ids, const ModuleConfig& config)
    : m_driveMotor(ids.drive.id, ids.drive.bus),
      m_steerMotor(ids.steer.id, ids.steer.bus),
      m_encoder(ids.encoder.id, ids.encoder.bus),
      m_config(config),
      m_driveVelocitySignal(m_driveMotor.GetVelocity()),
      m_drivePositionSignal(m_driveMotor.GetPosition()),
      m_driveCurrentSignal(m_driveMotor.GetSupplyCurrent()),
      m_driveTorqueCurrentSignal(m_driveMotor.GetTorqueCurrent()),
      m_driveVoltageSignal(m_driveMotor.GetMotorVoltage()),
      m_driveTempSignal(m_driveMotor.GetDeviceTemp()),
      m_steerPositionSignal(m_steerMotor.GetPosition()),
      m_steerVelocitySignal(m_steerMotor.GetVelocity()),
      m_steerVoltageSignal(m_steerMotor.GetMotorVoltage()),
      m_steerTempSignal(m_steerMotor.GetDeviceTemp()),
      m_encoderPositionSignal(m_encoder.GetAbsolutePosition()),
      m_synchedSignals{&m_driveVelocitySignal, &m_drivePositionSignal,
                       &m_steerPositionSignal, &m_steerVelocitySignal,
                       &m_encoderPositionSignal},
      m_batchedSignals{
          &m_driveCurrentSignal, &m_driveTorqueCurrentSignal,
          &m_driveVoltageSignal, &m_driveTempSignal,
          &m_steerVoltageSignal, &m_steerTempSignal,
      } {
  ConfigureDevices();
  ConfigureSignalFrequencies();
}

void CTREModuleIO::ConfigureDevices() {
  // DRIVE
  m_driveConfig.MotorOutput.Inverted =
      kDriveInverted ? signals::InvertedValue::Clockwise_Positive
                     : signals::InvertedValue::CounterClockwise_Positive;
  m_driveConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

  m_driveConfig.Voltage.PeakForwardVoltage = 12_V;
  m_driveConfig.Voltage.PeakReverseVoltage = -12_V;

  m_driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = kDriveSlipCurrent;
  m_driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -kDriveSlipCurrent;

  m_driveConfig.CurrentLimits.StatorCurrentLimit = kDriveSlipCurrent;
  m_driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

  m_driveConfig.CurrentLimits.SupplyCurrentLimit = kDriveSupplyCurrentLimit;
  m_driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

  m_steerConfig.MotorOutput.Inverted =
      kSteerInverted ? signals::InvertedValue::Clockwise_Positive
                     : signals::InvertedValue::CounterClockwise_Positive;
  m_steerConfig.MotorOutput.NeutralMode = signals::NeutralModeValue::Brake;

  // Use fused CANcoder for absolute module angle
  m_steerConfig.Feedback.FeedbackRemoteSensorID = m_encoder.GetDeviceID();
  m_steerConfig.Feedback.FeedbackSensorSource =
      signals::FeedbackSensorSourceValue::FusedCANcoder;

  // Map integrated rotor to sensor with gear ratio so "Position" is mechanism
  // turns
  m_steerConfig.Feedback.RotorToSensorRatio = kSteerGearRatio;
  m_steerConfig.Feedback.SensorToMechanismRatio = 1.0;

  m_steerConfig.CurrentLimits.SupplyCurrentLimit = kSteerSupplyCurrentLimit;
  m_steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

  m_steerConfig.Voltage.PeakForwardVoltage = 12_V;
  m_steerConfig.Voltage.PeakReverseVoltage = -12_V;

  ConfigureClosedLoop();

  configs::CANcoderConfiguration encoderConfig{};
  encoderConfig.MagnetSensor.MagnetOffset = m_config.encoderOffset;
  m_encoder.GetConfigurator().Apply(encoderConfig);
  m_driveMotor.GetConfigurator().Apply(m_driveConfig);
  m_steerMotor.GetConfigurator().Apply(m_steerConfig);
}

void CTREModuleIO::UpdateInputs(ModuleIOInputs& inputs, bool isBatched) {
  // I can probably clean this up
  if (!isBatched) {  // If we dont use the blocking batched call, update inputs
    ctre::phoenix6::BaseStatusSignal::RefreshAll(m_synchedSignals);
  }
  ctre::phoenix6::BaseStatusSignal::RefreshAll(m_batchedSignals);

  const units::turn_t driveMotorPos =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          m_drivePositionSignal, m_driveVelocitySignal);  // motor rotor turns
  const units::turn_t steerPos =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          m_steerPositionSignal, m_steerVelocitySignal);  // mechanism turns

  const units::turn_t coupledDriveMotorPos =
      driveMotorPos - (steerPos * kCouplingRatio);

  const units::turns_per_second_t driveMotorVel =
      m_driveVelocitySignal.GetValue();

  constexpr double driveGearRatio = kDriveGearRatio;
  const units::turn_t driveWheelPos = coupledDriveMotorPos / driveGearRatio;
  const units::turns_per_second_t driveWheelVel =
      driveMotorVel / driveGearRatio;

  // kinematics: m/s = (turn/s) * (m/turn), m = (turn) * (m/turn)
  constexpr auto metersPerTurn = kMeterPerTurn;
  inputs.driveVelocity = driveWheelVel * metersPerTurn;
  inputs.drivePosition = driveWheelPos * metersPerTurn;

  const units::turn_t steerTurnsWrapped = WrapToHalfTurns(steerPos);
  const units::radian_t steerAngleRad = steerTurnsWrapped;
  inputs.steerAngle = frc::Rotation2d{WrapToPi(steerAngleRad)};
  inputs.steerPosition = steerTurnsWrapped;

  inputs.driveCurrentDraw = m_driveCurrentSignal.GetValue();
  inputs.driveTorque =
      kDriveMotor.Torque(m_driveTorqueCurrentSignal.GetValue());
  inputs.driveAppliedVolts = m_driveVoltageSignal.GetValue();
  inputs.steerAppliedVolts = m_steerVoltageSignal.GetValue();
  inputs.driveTemperature = m_driveTempSignal.GetValue();
  inputs.steerTemperature = m_steerTempSignal.GetValue();

  inputs.driveConnected = m_driveVelocitySignal.GetStatus().IsOK() &&
                          m_drivePositionSignal.GetStatus().IsOK();
  inputs.steerConnected = m_steerPositionSignal.GetStatus().IsOK();
  inputs.encoderConnected = m_encoderPositionSignal.GetStatus().IsOK();
}

void CTREModuleIO::SetDesiredState(const frc::SwerveModuleState& state) {
  // --- Steer: command in TURNS, enable wrap in config
  const units::turn_t targetSteerTurns = state.angle.Radians();
  m_steerMotor.SetControl(
      m_steerPosition.WithPosition(targetSteerTurns).WithSlot(0));

  // --- Drive: convert wheel velocity to motor rotor velocity
  constexpr auto metersPerTurn = kMeterPerTurn;
  constexpr auto driveGearRatio = kDriveGearRatio;

  // Convert desired wheel speed to wheel turns per second
  const units::turns_per_second_t wheelRps = state.speed / metersPerTurn;

  // Convert wheel turns per second to motor rotor turns per second
  const units::turns_per_second_t motorRps = wheelRps * driveGearRatio;

  m_driveMotor.SetControl(m_driveVelocity.WithVelocity(motorRps).WithSlot(0));
}

void CTREModuleIO::Stop() {
  m_driveMotor.SetControl(m_driveVoltage.WithOutput(0_V));
  m_steerMotor.SetControl(
      m_steerPosition.WithPosition(m_steerPositionSignal.GetValue()));
}

void CTREModuleIO::SetBrakeMode(bool brake) {
  auto neutralMode = brake ? signals::NeutralModeValue::Brake
                           : signals::NeutralModeValue::Coast;

  // Refresh (in case Tuner changed configs), then apply
  m_driveMotor.GetConfigurator().Refresh(m_driveConfig);
  m_steerMotor.GetConfigurator().Refresh(m_steerConfig);

  m_driveConfig.MotorOutput.NeutralMode = neutralMode;
  m_steerConfig.MotorOutput.NeutralMode = neutralMode;

  m_driveMotor.GetConfigurator().Apply(m_driveConfig);
  m_steerMotor.GetConfigurator().Apply(m_steerConfig);
}

void CTREModuleIO::ResetDriveEncoder() {
  m_driveMotor.SetPosition(0_tr);  // zero mechanism turns
}

ModuleConfig CTREModuleIO::GetModuleConfig() {
  return m_config;
}

void CTREModuleIO::ConfigureClosedLoop() {
  m_driveConfig.Slot0.kP = PID::DriveVelocity::kP;
  m_driveConfig.Slot0.kI = PID::DriveVelocity::kI;
  m_driveConfig.Slot0.kD = PID::DriveVelocity::kD;
  m_driveConfig.Slot0.kS = PID::DriveVelocity::kS;
  m_driveConfig.Slot0.kV = PID::DriveVelocity::kV;
  m_driveConfig.Slot0.kA = PID::DriveVelocity::kA;

  m_steerConfig.Slot0.kP = PID::SteerPosition::kP;
  m_steerConfig.Slot0.kI = PID::SteerPosition::kI;
  m_steerConfig.Slot0.kD = PID::SteerPosition::kD;
  m_steerConfig.Slot0.kS = PID::SteerPosition::kS;
  m_steerConfig.Slot0.kV = PID::SteerPosition::kV;
  m_steerConfig.Slot0.kA = PID::SteerPosition::kA;

  // Let Phoenix wrap the position error across +/- pi
  m_steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
}

void CTREModuleIO::ConfigureSignalFrequencies() {
  m_driveVelocitySignal.SetUpdateFrequency(250_Hz);
  m_drivePositionSignal.SetUpdateFrequency(250_Hz);
  m_driveCurrentSignal.SetUpdateFrequency(50_Hz);
  m_driveVoltageSignal.SetUpdateFrequency(50_Hz);
  m_driveTempSignal.SetUpdateFrequency(4_Hz);

  m_steerPositionSignal.SetUpdateFrequency(250_Hz);
  m_steerVelocitySignal.SetUpdateFrequency(250_Hz);
  m_steerVoltageSignal.SetUpdateFrequency(50_Hz);
  m_steerTempSignal.SetUpdateFrequency(4_Hz);

  m_encoderPositionSignal.SetUpdateFrequency(250_Hz);

  m_driveMotor.OptimizeBusUtilization();
  m_steerMotor.OptimizeBusUtilization();
}
