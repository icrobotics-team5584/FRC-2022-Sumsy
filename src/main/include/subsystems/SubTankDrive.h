// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "utilities/ICSparkMax.h"

class SubTankDrive : public frc2::SubsystemBase {
 public:
  static SubTankDrive &GetInstance() {static SubTankDrive inst; return inst;}

  void drive(double speed, double rotation, bool squaredInputs = false);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SetCoastMode(); 
  void SetBreakMode();

 private:
 SubTankDrive();
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ICSparkMax _spmFrontLeft{canid::spmDriveBaseFrontLeft};
  ICSparkMax _spmFrontRight{canid::spmDriveBaseFrontRight};
  ICSparkMax _spmBackLeft{canid::spmDriveBaseBackLeft};
  ICSparkMax _spmBackRight{canid::spmDriveBaseBackRight};
  //rev::CANSparkMax _spmDolly{canid::spmDolly, rev::CANSparkMax::MotorType::kBrushless};
  frc::DifferentialDrive _diffDrive{_spmFrontLeft, _spmFrontRight};

  unsigned int _currentLimit = 50;
};
