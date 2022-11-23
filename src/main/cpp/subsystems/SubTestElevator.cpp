// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubTestElevator.h"

SubTestElevator::SubTestElevator(){
    _motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
};

// This method will be called once per scheduler run
void SubTestElevator::Periodic() {}

void SubTestElevator::Up() {
    _motor.Set(0.5);
}

void SubTestElevator::Down() {
    _motor.Set(-0.5);
}

void SubTestElevator::Stop() {
    _motor.Set(0);
}