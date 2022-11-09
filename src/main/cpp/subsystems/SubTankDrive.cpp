// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubTankDrive.h"

SubTankDrive::SubTankDrive() {
    // Reset all the motor controllers to factory default
  _spmFrontLeft.RestoreFactoryDefaults();
  _spmFrontRight.RestoreFactoryDefaults();
  _spmBackLeft.RestoreFactoryDefaults();
  _spmBackRight.RestoreFactoryDefaults();

	// Set Current Limiting on all drive motors
  _spmFrontLeft.SetSmartCurrentLimit(_currentLimit);
  _spmFrontRight.SetSmartCurrentLimit(_currentLimit);
  _spmBackLeft.SetSmartCurrentLimit(_currentLimit);
  _spmBackRight.SetSmartCurrentLimit(_currentLimit);

	// make back left follow front left and make back right follow front right
  _spmBackLeft.Follow(_spmFrontLeft);
  _spmBackRight.Follow(_spmFrontRight);

  _spmFrontRight.SetInverted(true);

  SetBreakMode();
}

void SubTankDrive::drive(double speed, double rotation, bool squaredInputs){
  _diffDrive.ArcadeDrive(speed, rotation, squaredInputs);
}

// This method will be called once per scheduler run
void SubTankDrive::Periodic() {}

void SubTankDrive::SetBreakMode() {
  _spmFrontLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  _spmFrontRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  _spmBackLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  _spmBackRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void SubTankDrive::SetCoastMode() {
  _spmFrontLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  _spmFrontRight.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  _spmBackLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  _spmBackRight.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}
