// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"
#include <frc/smartdashboard/SmartDashboard.h>
//SubIntake::SubIntake() = default;
//line 7 doesnt compile while active, must be //ed out to build

// This method will be called once per scheduler run
void SubIntake::Periodic() {
    frc::SmartDashboard::PutNumber("IntakeSpeed", _spmIntakeSpin.Get());
}
void SubIntake::Intake(){
    _spmIntakeSpin.Set(1);
} 
void SubIntake::Outtake(){
    _spmIntakeSpin.Set(-1);
}
void SubIntake::Stop(){
    _spmIntakeSpin.Set(0);
}


void SubIntake::Extend(){
    _solIntakeDeploy.Set(frc::DoubleSolenoid::Value::kForward);
}
void SubIntake::Retract(){
    _solIntakeDeploy.Set(frc::DoubleSolenoid::Value::kReverse);
}
