#pragma once

#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <frc/simulation/SimDeviceSim.h>
#include <hal/SimDevice.h>
#include <frc/Notifier.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>

class ICSparkMax : public rev::CANSparkMax {
 public:
  enum class Type { NEO, NEO_550 };

  ICSparkMax(int deviceID, Type type);

  void SetPIDF(double P, double I, double D, double F);

  void SetPositionConversionFactor(double factor);
  void SetVelocityConversionFactor(double factor);

  void SetPositionTarget(double target);
  void SetSmartMotionTarget(double target);
  void SetVelocityTarget(double target);

  bool GoingForward();
  bool GoingBackward();

  units::volt_t GetSimVoltage();

  rev::SparkMaxRelativeEncoder& GetEncoderRef() { return _encoder; };
  rev::SparkMaxPIDController& GetPIDControllerRed() { return _pidController; };

  rev::ControlType GetControlType() { return _controlType; };

  void StopMotor() override;

 private:
  double _target;
  Type _type;
  rev::ControlType _controlType;
  unsigned int NEO_CURRENT_LIMIT = 40;
  unsigned int NEO_550_CURRENT_LIMIT = 20;

  rev::SparkMaxPIDController _pidController{GetPIDController()};
  rev::SparkMaxRelativeEncoder _encoder{GetEncoder()};

  // frc::ProfiledPIDController<units::meters> _simSmartMotionController{0,0,0,{1.75_mps, 0.75_mps_sq},20_ms};
  frc::PIDController _simVelocityController{0,0,0};
};
