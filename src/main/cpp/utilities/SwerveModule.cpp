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
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  auto targetState = frc::SwerveModuleState::Optimize(referenceState, GetAngle());

  // Move target angle so we can cross over the 180 degree line without going the long way round
  frc::Rotation2d difference = targetState.angle - GetAngle();
  difference = frc::InputModulus(difference.Degrees(), -180_deg, 180_deg);
  auto targetAngle = GetAngle().Degrees() + difference.Degrees();

  frc::SmartDashboard::PutNumber("Target after move", targetAngle.value());

  // Calculate the drive output from the drive PID controller.
  //const auto driveOutput = m_drivePIDController.Calculate(_canDriveMotor.GetSelectedSensorVelocity()*kRotationConversion, state.speed.value());
  
  //_canDriveMotor.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput, (driveOutput + (double) driveFeedforward));
  SetDesiredAngle(targetAngle);
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