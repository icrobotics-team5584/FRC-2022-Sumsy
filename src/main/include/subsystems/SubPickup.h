// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DoubleSolenoid.h>
#include <ctre/Phoenix.h>
#include "Constants.h"

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

  void Intake();

  void Outake();

  void Stoptake();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  frc::DoubleSolenoid _piston {1, frc::PneumaticsModuleType::CTREPCM, pcm::solIntakeOut, pcm::solIntakeIn};
  WPI_TalonFX _intakeMotor {canid::tfxIntake};
};
