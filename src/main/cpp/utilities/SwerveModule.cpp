#include "utilities/SwerveModule.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/MathUtil.h>

SwerveModule::SwerveModule(int canDriveMotorID, int canTurnMotorID, int canTurnEncoderID) 
                          : _canDriveMotor(canDriveMotorID), 
                          _canTurnMotor(canTurnMotorID),
                          _canTurnEncoder(canTurnEncoderID){

  // Config CANCoder
  _canTurnEncoder.ConfigFactoryDefault();
  _canTurnEncoder.SetPositionToAbsolute();
  _canTurnEncoder.ConfigAbsoluteSensorRange(ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180);

  // Config Turning Motor
  _canTurnMotor.ConfigFactoryDefault();
  _canTurnMotor.ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Signed_PlusMinus180);
  _canTurnMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
  _canTurnMotor.ConfigFeedbackNotContinuous(true);
  _canTurnMotor.Config_kP(PID_SLOT_INDEX, TURN_P);
  _canTurnMotor.Config_kI(PID_SLOT_INDEX, TURN_I);
  _canTurnMotor.Config_kD(PID_SLOT_INDEX, TURN_D);

  // Config Driving Motor
  _canDriveMotor.ConfigFactoryDefault();
  _canDriveMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
  _canDriveMotor.Config_kP(PID_SLOT_INDEX, DRIVE_P);
  _canDriveMotor.Config_kI(PID_SLOT_INDEX, DRIVE_I);
  _canDriveMotor.Config_kD(PID_SLOT_INDEX, DRIVE_D);
  _canDriveMotor.Config_kF(PID_SLOT_INDEX, DRIVE_F);
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  auto targetState = frc::SwerveModuleState::Optimize(referenceState, GetAngle());

  // Move target angle so we can cross over the 180 degree line without going the long way round
  frc::Rotation2d difference = targetState.angle - GetAngle();
  difference = frc::InputModulus(difference.Degrees(), -180_deg, 180_deg);
  targetState.angle = GetAngle() + difference;

  // Drive! These functions do some conversions and send targets to falcons
  SetDesiredAngle(targetState.angle);
  SetDesiredVelocity(targetState.speed);
}

void SwerveModule::SendSensorsToDash() {
  const int driveMotorID =  _canDriveMotor.GetDeviceID();
  const int turnMotorID =  _canTurnMotor.GetDeviceID();
  const int turnEncoderID =  _canTurnEncoder.GetDeviceNumber();

  frc::SmartDashboard::PutNumber("Drive motor "+std::to_string(driveMotorID)+ " velocity", _canDriveMotor.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber("Turn motor "+std::to_string(turnMotorID)+ " position tics", _canTurnMotor.GetSensorCollection().GetIntegratedSensorPosition());
  frc::SmartDashboard::PutNumber("Turn motor "+std::to_string(turnMotorID)+ " position degrees", GetAngle().Degrees().value());
  frc::SmartDashboard::PutNumber("Turn motor "+std::to_string(turnMotorID)+ " target", _canTurnMotor.GetClosedLoopTarget());
  frc::SmartDashboard::PutNumber("Turn motor "+std::to_string(turnMotorID)+ " error", _canTurnMotor.GetClosedLoopError());
  frc::SmartDashboard::PutNumber("Turn motor "+std::to_string(turnMotorID)+ " selected sensor pos", _canTurnMotor.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("Turn encoder "+std::to_string(turnEncoderID)+ " Abs position", _canTurnEncoder.GetAbsolutePosition());
  frc::SmartDashboard::PutNumber("Turn encoder "+std::to_string(turnEncoderID)+ " position", _canTurnEncoder.GetPosition());
}

frc::Rotation2d SwerveModule::GetAngle() {
  const double positionInTics = _canTurnMotor
    .GetSensorCollection()
    .GetIntegratedSensorPosition();
  const double positionInRevolutions = positionInTics / TICS_PER_TURNING_WHEEL_REVOLUTION;
  const double positionInDegrees = positionInRevolutions * 360;
  return frc::Rotation2d(units::degree_t(positionInDegrees));
}

void SwerveModule::SetDesiredAngle(frc::Rotation2d angle) {
  const double targetDegrees = angle.Degrees().value();
  const double targetRotations = targetDegrees / 360.0;
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

void SwerveModule::ZeroSensors() {
  
}

void SwerveModule::SyncSensors() {
  double cancoderDegrees = _canTurnEncoder.GetAbsolutePosition();
  double cancoderRevolutions = cancoderDegrees/360;
  int cancoderPosInFalconTics = cancoderRevolutions*TICS_PER_TURNING_WHEEL_REVOLUTION;

  _canTurnMotor
    .GetSensorCollection()
    .SetIntegratedSensorPosition(cancoderPosInFalconTics);
}