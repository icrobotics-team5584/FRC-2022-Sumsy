// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DoubleSolenoid.h>
#include <ctre/Phoenix.h>

class SubPickup : public frc2::SubsystemBase {
 public:
  SubPickup();

  static SubPickup &GetInstance() {static SubPickup inst; return inst;}

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Extender();

  void Retractor();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  frc::DoubleSolenoid _piston {frc::PneumaticsModuleType::CTREPCM, 0, 1};

};
