// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DoubleSolenoid.h>
#include <rev/CANSparkMax.h>

class SubIntake : public frc2::SubsystemBase {
 public:
  void Periodic() override;
  void Intake();
  void Outtake();
  void Stop();
  void Extend();
  void Retract();

 private:
  rev::CANSparkMax _spmIntakeSpin{9, rev::CANSparkMaxLowLevel::MotorType::kBrushless}; 
  frc::DoubleSolenoid _solIntakeDeploy {frc::PneumaticsModuleType::CTREPCM, 1, 2};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
