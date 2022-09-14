// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include <frc/DigitalInput.h>

class SubPayload : public frc2::SubsystemBase {
 public:
  SubPayload();

  static SubPayload &GetInstance() {static SubPayload inst; return inst;}

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Intake();

  void Outake();

  void Stop();

  bool HasBall();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::CANSparkMax _spmPayload{canid::spmPayload, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  frc::DigitalInput _LineBreakPayload{dio::lineBreakPayload};

};
