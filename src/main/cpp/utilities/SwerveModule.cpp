#include "utilities/SwerveModule.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/MathUtil.h>

SwerveModule::SwerveModule(int canDriveMotorID, int canTurnMotorID,
                           int canTurnEncoderID, double cancoderMagOffset)
    : _canDriveMotor(canDriveMotorID),
      _canTurnMotor(canTurnMotorID),
      _canTurnEncoder(canTurnEncoderID) {
  // Config CANCoder
  _canTurnEncoder.ConfigFactoryDefault();
  _canTurnEncoder.SetPositionToAbsolute();
  _canTurnEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  _canTurnEncoder.ConfigMagnetOffset(cancoderMagOffset);
  _canTurnEncoder.ConfigSensorDirection(true); // Invert to match rotation of falcon internal sensors

  // Config Turning Motor
  _canTurnMotor.ConfigFactoryDefault();
  _canTurnMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
  _canTurnMotor.ConfigFeedbackNotContinuous(true);
  _canTurnMotor.Config_kP(PID_SLOT_INDEX, TURN_P);
  _canTurnMotor.Config_kI(PID_SLOT_INDEX, TURN_I);
  _canTurnMotor.Config_kD(PID_SLOT_INDEX, TURN_D);
  _canTurnMotor.ConfigSupplyCurrentLimit(CURRENT_LIMIT_CONFIG);

  // Config Driving Motor
  _canDriveMotor.ConfigFactoryDefault();
  _canDriveMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
  _canDriveMotor.Config_kP(PID_SLOT_INDEX, DRIVE_P);
  _canDriveMotor.Config_kI(PID_SLOT_INDEX, DRIVE_I);
  _canDriveMotor.Config_kD(PID_SLOT_INDEX, DRIVE_D);
  _canDriveMotor.Config_kF(PID_SLOT_INDEX, DRIVE_F);
  _canTurnMotor.ConfigSupplyCurrentLimit(CURRENT_LIMIT_CONFIG);
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  auto targetState = frc::SwerveModuleState::Optimize(referenceState, GetAngle());

  // Move target angle so we can cross over the 180 degree line without going the long way round
  auto difference = targetState.angle.Degrees() - GetAngle().Degrees();
  difference = frc::InputModulus(difference, -180_deg, 180_deg);
  auto targetAngle = GetAngle().Degrees() + difference;

  // Drive! These functions do some conversions and send targets to falcons
  SetDesiredAngle(targetAngle);
  SetDesiredVelocity(targetState.speed);
}

void SwerveModule::SendSensorsToDash() {
  std::string driveMotorName = "drive motor " + std::to_string(_canDriveMotor.GetDeviceID());
  std::string turnMotorName = "turn motor " + std::to_string(_canTurnMotor.GetDeviceID());
  std::string turnEncoderName = "turn encoder " + std::to_string(_canTurnEncoder.GetDeviceNumber());

  frc::SmartDashboard::PutNumber(driveMotorName + " velocity", _canDriveMotor.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber(turnMotorName  + " position degrees", GetAngle().Degrees().value());
  frc::SmartDashboard::PutNumber(turnMotorName  + " position tics", _canTurnMotor.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber(turnMotorName  + " target", _canTurnMotor.GetClosedLoopTarget());
  frc::SmartDashboard::PutNumber(turnMotorName  + " error", _canTurnMotor.GetClosedLoopError());
  frc::SmartDashboard::PutNumber(turnEncoderName+ " Abs position", _canTurnEncoder.GetAbsolutePosition());
}

frc::Rotation2d SwerveModule::GetAngle() {
  const double positionInTics = _canTurnMotor.GetSelectedSensorPosition();
  const double positionInRevolutions = positionInTics / TICS_PER_TURNING_WHEEL_REVOLUTION;
  const double positionInDegrees = positionInRevolutions * 360;
  return frc::Rotation2d(units::degree_t(positionInDegrees));
}

void SwerveModule::SetDesiredAngle(units::degree_t angle) {
  const double targetRotations = angle.value() / 360.0;
  const int targetTics =  targetRotations * TICS_PER_TURNING_WHEEL_REVOLUTION;
  _canTurnMotor.Set(TalonFXControlMode::Position, targetTics);
}

void SwerveModule::SetDesiredVelocity(units::meters_per_second_t velocity) {
  // Must convert from meters per second to encoder tics per 100ms, ouch.
  const double metersPerMS = velocity.value() / 1000;
  const double metersPer100MS = metersPerMS * 100;
  const double revolutionsPer100MS = metersPer100MS / WHEEL_RADIUS.value();
  const double ticsPer100MS = revolutionsPer100MS * TICS_PER_MOTOR_REVOLUTION;
  _canDriveMotor.Set(TalonFXControlMode::Velocity, ticsPer100MS);
}

void SwerveModule::StopMotors() {
  _canDriveMotor.Set(TalonFXControlMode::PercentOutput, 0);
  _canTurnMotor.Set(TalonFXControlMode::PercentOutput, 0);
}

void SwerveModule::SyncSensors() {
  double cancoderDegrees = _canTurnEncoder.GetAbsolutePosition();
  double cancoderRevolutions = cancoderDegrees/360;
  int cancoderPosInFalconTics = cancoderRevolutions*TICS_PER_TURNING_WHEEL_REVOLUTION;

  // _canTurnMotor
  //   .GetSensorCollection()
  //   .SetIntegratedSensorPosition(cancoderPosInFalconTics);
  _canTurnMotor.SetSelectedSensorPosition(cancoderPosInFalconTics);
}