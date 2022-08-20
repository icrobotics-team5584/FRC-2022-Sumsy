#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <rev/CANSparkMax.h>
#include "utilities/ICSparkMax.h"
#include "Constants.h"
#include <frc/simulation/ElevatorSim.h>
#include <frc/controller/ElevatorFeedforward.h>

class SubElevator : public frc2::SubsystemBase {
 public:
  static SubElevator &GetInstance() {static SubElevator inst; return inst;}

  void Periodic() override;
  void SimulationPeriodic() override;

  void DriveTo(units::meter_t height);
  void DriveAt(units::meters_per_second_t velocity);
  void DriveWith(units::volt_t voltage);
  void DumbDriveTo(units::meter_t height);
  void DumbDriveAt(double power);

  void Stop();

 private:
  SubElevator();

  // Electronics
  ICSparkMax _elevator{canid::tfxElevator, ICSparkMax::Type::NEO};

  // Constants
  double P = 0.0;
  double I = 0.0;
  double D = 0.0;
  frc::ElevatorFeedforward<units::meters> _elevatorFF {
    0.0_V,
    0.005_V,
    12_V/0.3_mps
  };

  const double ELEVATOR_GEARING = 20.0;
  const units::meter_t DRUM_RADIUS = 0.05_m;
  const double ELEVATOR_POS_CONVERSION_FACTOR = ELEVATOR_GEARING * DRUM_RADIUS.value(); // rotations to meters
  const double ELEVATOR_VEL_CONVERSION_FACTOR = ELEVATOR_POS_CONVERSION_FACTOR * 60; // RPM to meters per second
  const units::meter_t MIN_HEIGHT = 0.001_m;
  const units::meter_t MAX_HEIGHT = 1.5_m;
  const units::kilogram_t CARRIAGE_MASS = 0.01_kg; // Assume constant force springs do a great job at canceling out gravity, near zero weight.

  // Simulation
  frc::sim::ElevatorSim _sim{
    frc::DCMotor::NEO(),
    ELEVATOR_GEARING,
    CARRIAGE_MASS,
    DRUM_RADIUS,
    MIN_HEIGHT,
    MAX_HEIGHT
  };
};
