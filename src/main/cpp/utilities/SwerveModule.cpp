// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utilities/SwerveModule.h"

SwerveModule::SwerveModule(int canDriveMotorID, int canTurnMotorID, int canTurnEncoderID) 
                          : _canDriveMotor(canDriveMotorID), 
                          _canTurnMotor(canTurnMotorID),
                           _canTurnEncoder(canTurnEncoderID){

  m_turningPIDController.EnableContinuousInput(
    -units::radian_t(wpi::numbers::pi), units::radian_t(wpi::numbers::pi));
  _canTurnEncoder.SetPositionToAbsolute();
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return (units::meters_per_second_t{_canDriveMotor.GetSelectedSensorVelocity(0)*10/},
          frc::Rotation2d(units::radian_t(_canTurnMotor.Get)))
}