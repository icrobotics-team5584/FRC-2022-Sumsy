// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Constants.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/velocity.h>

class SubElevator : public frc2::SubsystemBase {
 public:
  static SubElevator& Getinstance() {
    static SubElevator inst;
    return inst;
  }

  SubElevator();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Extendfirst();   // extends the elevators to the first position
  void Extendsecond();  // extends the evevators to the second position
  void Extendthird();   // extends the elevators to the third position
  void Retract();       // retracts the elevators one position
  void Reset();         // resets the evevators to the lowest position

 private:
 
  static constexpr units::meter_t MIN_POSITION = 0_m;
  static constexpr units::meter_t FIRST_POSITION = 0.5_m;
  static constexpr units::meter_t SECOND_POSITION = 0.75_m;
  static constexpr units::meter_t THIRD_POSITION = 1_m;
  const units::meters_per_second_t MAX_VELOCITY = 1_mps;
  const units::meters_per_second_squared_t MAX_ACCERLATION = 1_mps_sq;

  static constexpr double kP = 5e-5, kI = 1e-6, kD = 0;
  const double GEARBOX_REDUCTION = 20;
  const units::meter_t PULLY_DIAMETER = 0.002_m;
  const double CONVERSTION_FACTOR = GEARBOX_REDUCTION * PULLY_DIAMETER.value();

  rev::CANSparkMax _spmLeftElevator{canid::spmElevatorLeft,
                                    rev::CANSparkMax::MotorType::kBrushless};

  rev::CANSparkMax _spmRightElevator{canid::spmElevatorRight,
                                     rev::CANSparkMax::MotorType::kBrushless};

  rev::SparkMaxPIDController _pidLeftMotorController =
      _spmLeftElevator.GetPIDController();

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
