// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SubPayload.h"
#include "Constants.h"

SubPayload::SubPayload(){
    _spmPayloadFollow.Follow(_spmPayload, false);
}

// This method will be called once per scheduler run
void SubPayload::Periodic() {
    frc::SmartDashboard::PutNumber("Payload motor speed", _spmPayload.Get());
    frc::SmartDashboard::PutNumber("PayloadFollow motor speed", _spmPayloadFollow.Get());
    frc::SmartDashboard::PutBoolean("Payload linebreak", HasBall());  
}

void SubPayload::Intake() {
    _spmPayload.Set(0.3);
}

void SubPayload::Outake() {
    _spmPayload.Set(-0.3);
}

void SubPayload::Stop() {
    _spmPayload.Set(0.0);
}

bool SubPayload::HasBall() {
    return _LineBreakPayload.Get();
}
