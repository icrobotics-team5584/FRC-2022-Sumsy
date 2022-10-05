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
#include <utilities/ICSparkMax.h>
#include <frc/simulation/ElevatorSim.h>
#include <units/mass.h>
#include <frc/DigitalInput.h>
#include <rev/SparkMaxRelativeEncoder.h>
#include <utilities/ICSparkMax.h>

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
  void SimulationPeriodic() override;

  void Extendfirst();   // extends the elevators to the first position
  void Extendsecond();  // extends the evevators to the second position
  void Extendthird();   // extends the elevators to the third position
  void Retract();       // retracts the elevators one position
  void Reset();         // resets the evevators to the lowest position
  bool AtLowerLimit(); //Check if lower limit switch is hit
  bool AtUpperLimit(); //Check if upper limit switch is hit
  bool GoingUp(); //check if the target position is above current position
  bool GoingDown(); //Check if the target position is below current position


  static constexpr units::meter_t MIN_POSITION = 0_m;
  static constexpr units::meter_t FIRST_POSITION = 0.5_m;
  static constexpr units::meter_t SECOND_POSITION = 0.75_m;
  static constexpr units::meter_t THIRD_POSITION = 1_m;

 private:
 
  
  const units::meters_per_second_t MAX_VELOCITY = 0.1_mps;
  const units::meters_per_second_squared_t MAX_ACCERLATION = 0.1_mps_sq;
  const units::kilogram_t CARRAGE_WEIGHT = 0.01_kg; 

  static constexpr double kP = 0.0, kI = 0, kD = 0, kFF = 4.7;
  const double GEARBOX_REDUCTION = 20;
  const units::meter_t DRUM_RADIUS = 0.05_m;
  const double POS_CONVERSION_FACTOR = GEARBOX_REDUCTION * DRUM_RADIUS.value();
  const double VEL_CONVERSION_FACTOR = POS_CONVERSION_FACTOR / 60;

   ICSparkMax _spmLeftElevator{canid::spmElevatorLeft,ICSparkMax::Type::NEO};

  ICSparkMax _spmRightElevator{canid::spmElevatorRight,ICSparkMax::Type::NEO};

//simulation code:
frc::sim::ElevatorSim _elevatorSim{frc::DCMotor::NEO(2),GEARBOX_REDUCTION,CARRAGE_WEIGHT,DRUM_RADIUS,MIN_POSITION,THIRD_POSITION};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::DigitalInput _Lowerlmt{dio::lmtLowerElevator};
  frc::DigitalInput _Upperlmt{dio::lmtUpperElevator};

  bool _inSmartMotionMode = false;
  int _targetPosition = 0;

  
};
