#pragma once

#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/time.h>
#include <frc/Notifier.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/PIDController.h>
#include <frc/simulation/SimDeviceSim.h>
#include <hal/simulation/SimDeviceData.h>

class ICSparkMax : public rev::CANSparkMax {
 public:
  enum class Type { NEO, NEO_550 };

  ICSparkMax(int deviceID, Type type);

  void SetPIDF(double P, double I, double D, double F = 0.0);

  void SetTarget(double target, rev::ControlType controlType, int pidSlot = 0,
                 double arbFeedForward = 0.0,
                 rev::CANPIDController::ArbFFUnits arbFFUnits =
                     rev::CANPIDController::ArbFFUnits::kVoltage);

  units::volt_t GetSimVoltage();
  void SyncSimPID();
  void UpdateSimEncoder(double position);

  rev::SparkMaxRelativeEncoder& GetEncoderRef() { return _encoder; };
  rev::SparkMaxPIDController& GetPIDControllerRed() { return _pidController; };

  rev::ControlType GetControlType() { return _controlType; };

  void StopMotor() override;

 private:
  Type _type;
  rev::ControlType _controlType = rev::ControlType::kDutyCycle;
  rev::CANPIDController::ArbFFUnits _arbFFUnits = rev::CANPIDController::ArbFFUnits::kVoltage;
  double _arbFeedForward = 0.0;
  double _target = 0;
  int _pidSlot = 0;
 
  unsigned const int NEO_CURRENT_LIMIT = 40;
  unsigned const int NEO_550_CURRENT_LIMIT = 20;

  rev::SparkMaxPIDController _pidController{GetPIDController()};
  rev::SparkMaxRelativeEncoder _encoder{GetEncoder()};
  double _prevEncoderPos = 0.0;

  frc::TrapezoidProfile<units::meters> _simSmartMotionProfile{
    {0_mps, 0_mps_sq}, // constraints
    {0_m, 0_mps}       // goal states
  };
  frc::PIDController _simController{0,0,0};
  units::millisecond_t _timeSinceSmartMotionStart = 0_ms;
  frc::Notifier _smartMotionProfileNotifier{[&]{_timeSinceSmartMotionStart+=20_ms;}};
  double _simFF = 0;

  frc::sim::SimDeviceSim _simDeviceSim {"SPARK MAX ", GetDeviceId()};
  hal::SimDouble _simVelocity = _simDeviceSim.GetDouble("Velocity");
  hal::SimDouble _simPosition = _simDeviceSim.GetDouble("Position");
  hal::SimInt _simControlMode = _simDeviceSim.GetInt("Control Mode");

  void SetInternalControlType(rev::ControlType controlType);
};
