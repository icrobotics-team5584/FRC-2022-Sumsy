#pragma once

#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc/simulation/SimDeviceSim.h>
#include <hal/SimDevice.h>
#include <frc/Notifier.h>

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

 private:
  double _target;
  Type _type;
  rev::ControlType _controlType;
  unsigned int NEO_CURRENT_LIMIT = 40;
  unsigned int NEO_550_CURRENT_LIMIT = 20;

  rev::SparkMaxPIDController _pidController{GetPIDController()};
  rev::SparkMaxRelativeEncoder _encoder{GetEncoder()};
};
