#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/velocity.h>
#include <rev/CANSparkMax.h>
#include "utilities/ICSparkMax.h"
#include "Constants.h"
#include <frc/simulation/ElevatorSim.h>

class SubElevator : public frc2::SubsystemBase {
 public:
  static SubElevator &GetInstance() {static SubElevator inst; return inst;}

  void Periodic() override;
  void SimulationPeriodic() override;

  void DriveTo(units::meter_t height);
  void DriveAt(units::meters_per_second_t velocity);
  void Stop();

 private:
  SubElevator();

  // Electronics
  ICSparkMax _elevator{canid::tfxElevator, ICSparkMax::Type::NEO};

  // Constants
  const double P = 0.5;
  const double I = 0.0;
  const double D = 0.0;
  const double F = 0.1;

  const double ELEVATOR_GEARING = 30.0;
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
