// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

class SubTestElevator : public frc2::SubsystemBase {
 public:
  SubTestElevator();

  static SubTestElevator &GetInstance() {static SubTestElevator inst; return inst;}
  
  void Periodic() override;

  void Up();
  void Down();
  void Stop();

 private:
  rev::CANSparkMax _motor {20, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
};
