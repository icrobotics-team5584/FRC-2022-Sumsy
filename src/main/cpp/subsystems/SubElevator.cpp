// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubElevator.h"

SubElevator::SubElevator() {
    _pidLeftMotorController.SetP(kP);
    _pidLeftMotorController.SetI(kI);
    _pidLeftMotorController.SetD(kD);
    _pidLeftMotorController.SetSmartMotionMaxAccel(MAX_ACCERLATION.value());
    _pidLeftMotorController.SetSmartMotionMaxVelocity(MAX_VELOCITY.value());

    _spmLeftElevator.GetEncoder().SetPositionConversionFactor(CONVERSTION_FACTOR);

    _spmRightElevator.Follow(_spmLeftElevator);
    _spmRightElevator.SetSmartCurrentLimit(40);
    _spmLeftElevator.SetSmartCurrentLimit(40);
};


// This method will be called once per scheduler run
void SubElevator::Periodic() {}


void SubElevator::Extendfirst() {   
  _pidLeftMotorController.SetReference(FIRST_POSITION.value(), rev::ControlType::kSmartMotion);
}

