// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubPickup.h"
#include <ctre/Phoenix.h>

SubPickup::SubPickup() = default;

// This method will be called once per scheduler run
void SubPickup::Periodic() {

}

void SubPickup::Extender() {
    _piston.Set(frc::DoubleSolenoid::Value::kForward);
}

void SubPickup::Retractor() {
    _piston.Set(frc::DoubleSolenoid::Value::kReverse);
}

void SubPickup::Intake() {
    _intakeMotor.Set(1);
}

void SubPickup::Outake() {
    _intakeMotor.Set(-1);
}

void SubPickup::Stoptake() {
    _intakeMotor.Set(0);
}