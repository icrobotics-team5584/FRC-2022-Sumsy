// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utilities/FalconFactory.h"

FalconFactory::FalconFactory() = default;

std::unique_ptr<TalonFX> FalconFactory::MakePIDFalcon(int canID, double P, double I, double D) {
    auto falcon = std::make_unique<TalonFX>(canID);
    int pidSlotIndex = 0;
    falcon->ConfigFactoryDefault();
    falcon->Config_kP(pidSlotIndex, P);
    falcon->Config_kI(pidSlotIndex, I);
    falcon->Config_kD(pidSlotIndex, D);
    return falcon;
}