#include "utilities/ICSparkMax.h"

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/voltage.h>

#include <cstdlib>
#include <iostream>

ICSparkMax::ICSparkMax(int deviceID, Type type)
    : rev::CANSparkMax(deviceID,
                       rev::CANSparkMaxLowLevel::MotorType::kBrushless) {
  _type = type;
  RestoreFactoryDefaults();
  if (type == Type::NEO) {
    SetSmartCurrentLimit(NEO_CURRENT_LIMIT);
  } else {
    SetSmartCurrentLimit(NEO_550_CURRENT_LIMIT);
  }

  _pidController.SetSmartMotionMinOutputVelocity(0);
}

void ICSparkMax::InitSendable(wpi::SendableBuilder& builder) {
  builder.AddDoubleProperty(
      "Position", [&] { return _encoder.GetPosition(); },
      nullptr);  // setter is null, cannot set position directly
  builder.AddDoubleProperty(
      "Velocity", [&] { return _encoder.GetVelocity(); }, nullptr);
  builder.AddDoubleProperty(
      "Voltage", [&] { return GetSimVoltage().value(); }, nullptr);
  builder.AddBooleanProperty(
      "Update Target", [&] { return _updatingTargetFromSendable; },
      [&](bool update) { _updatingTargetFromSendable = update; });
  builder.AddDoubleProperty(
      "Target", [&] { return GetTarget(); },
      [&](double target) {
        if (_updatingTargetFromSendable) {
          SetTarget(target, _controlType);
          _updatingTargetFromSendable = false;
        }
      });
  builder.AddDoubleProperty(
      "Smart Motion Velocity Target", [&] { return GetCurrentSMVelocity(); },
      nullptr);
  builder.AddDoubleProperty(
      "P Gain", [&] { return _pidController.GetP(); },
      [&](double P) { _pidController.SetP(P); });
  builder.AddDoubleProperty(
      "I Gain", [&] { return _pidController.GetI(); },
      [&](double I) { _pidController.SetI(I); });
  builder.AddDoubleProperty(
      "D Gain", [&] { return _pidController.GetD(); },
      [&](double D) { _pidController.SetD(D); });
  builder.AddDoubleProperty(
      "F Gain", [&] { return _pidController.GetFF(); },
      [&](double F) { _pidController.SetFF(F); });
}

void ICSparkMax::SetTarget(double target, Mode controlType, int pidSlot,
                           double arbFeedForward) {
  _target = target;
  _pidSlot = pidSlot;
  _arbFeedForward = arbFeedForward;
  SetInternalControlType(controlType);

  _pidController.SetReference(target, controlType, pidSlot, _arbFeedForward);
  SyncSimPID();
}

void ICSparkMax::Set(double speed) {
  if (frc::RobotBase::IsSimulation()) {
    SetTarget(speed, Mode::kDutyCycle);
  }
  CANSparkMax::Set(speed);
}

void ICSparkMax::SetVoltage(units::volt_t output) {
  SetTarget(output.value(), Mode::kVoltage);
}

void ICSparkMax::SetSmartMotionMaxAccel(double maxAcceleration) {
  _pidController.SetSmartMotionMaxAccel(maxAcceleration /
                                        _RPMpsToDesiredAccelUnits);
}

void ICSparkMax::SetSmartMotionMaxVelocity(double maxVelocity) {
  _pidController.SetSmartMotionMaxVelocity(
      maxVelocity / _encoder.GetVelocityConversionFactor());
}

void ICSparkMax::SetConversionFactors(double rotationsToDesired,
                                      double RPMToDesired,
                                      double RPMpsToDesired) {
  _encoder.SetPositionConversionFactor(rotationsToDesired);
  _encoder.SetVelocityConversionFactor(RPMToDesired);
  _RPMpsToDesiredAccelUnits = RPMpsToDesired;
}

void ICSparkMax::SetPIDFF(double P, double I, double D, double FF) {
  _pidController.SetP(P);
  _pidController.SetI(I);
  _pidController.SetD(D);
  _pidController.SetFF(FF);
  SyncSimPID();
}

void ICSparkMax::SetEncoderPosition(double position) {
  _encoder.SetPosition(position);
}

void ICSparkMax::SetClosedLoopOutputRange(double minOutputPercent,
                                          double maxOutputPercent) {
  _pidController.SetOutputRange(minOutputPercent, maxOutputPercent);
}

units::volt_t ICSparkMax::GetSimVoltage() {
  units::volt_t output = 0_V;

  switch (_controlType) {
    case Mode::kDutyCycle:
      output = units::volt_t{_target * 12};
      break;

    case Mode::kVelocity:
      output = units::volt_t{
          _simController.Calculate(_encoder.GetVelocity(), _target) +
          _pidController.GetFF() * _target + _arbFeedForward};
      break;

    case Mode::kPosition:
      output = units::volt_t{
          _simController.Calculate(_encoder.GetPosition(), _target) +
          _pidController.GetFF() * _target + _arbFeedForward};
      break;

    case Mode::kVoltage:
      output = units::volt_t{_target};
      break;

    case Mode::kSmartMotion:
      output = units::volt_t{(_simController.Calculate(_encoder.GetVelocity(),
                                                       GetCurrentSMVelocity()) +
                              _pidController.GetFF() * GetCurrentSMVelocity() +
                              _arbFeedForward)};
      break;

    case Mode::kCurrent:
      std::cout << "Warning: closed loop Current control not supported by "
                   "ICSparkMax in Simulation\n";
      break;

    case Mode::kSmartVelocity:
      std::cout << "Warning: closed loop Smart Velocity control not supported "
                   "by ICSparkMax in Simulation\n";
      break;
  }
  return std::clamp(output, -12_V, 12_V);
}

void ICSparkMax::StopMotor() {
  _target = 0;
  SetInternalControlType(Mode::kDutyCycle);
  CANSparkMax::StopMotor();
}

void ICSparkMax::SetInternalControlType(Mode controlType) {
  _controlType = controlType;
  _simControlMode.Set((int)_controlType);
}

void ICSparkMax::UpdateSimEncoder(double position, double velocity) {
  _encoder.SetPosition(position);
  _simVelocity.Set(velocity);
  _prevEncoderPos = position;
}

void ICSparkMax::SyncSimPID() {
  if (frc::RobotBase::IsReal()) return;

  _simController.SetP(_pidController.GetP());
  _simController.SetI(_pidController.GetI());
  _simController.SetD(_pidController.GetD());
  _simController.SetIntegratorRange(-_pidController.GetIMaxAccum(),
                                    _pidController.GetIMaxAccum());

  if (_controlType == Mode::kSmartMotion) {
    GenerateSMProfile();
  } else {
    _smartMotionProfileTimer.Stop();
  }
}

void ICSparkMax::GenerateSMProfile() {
  _smartMotionProfileTimer.Reset();
  _smartMotionProfileTimer.Start();

  frc::TrapezoidProfile<units::meters>::Constraints constrainsts = {
      units::meters_per_second_t{_pidController.GetSmartMotionMaxVelocity() *
                                 _encoder.GetVelocityConversionFactor()},
      units::meters_per_second_squared_t{
          _pidController.GetSmartMotionMaxAccel() * _RPMpsToDesiredAccelUnits}};

  _simSmartMotionProfile = {
      constrainsts,
      {units::meter_t{_target}, 0_mps},
      {units::meter_t{_encoder.GetPosition()},
       units::meters_per_second_t{_encoder.GetVelocity()}}};
}

double ICSparkMax::GetCurrentSMVelocity() {
  if (frc::RobotBase::IsReal()) return 0.0;

  if (_smartMotionProfileTimer.Get() > 0.2_s) {
    GenerateSMProfile();
  }

  const auto error = std::abs(_target - _encoder.GetPosition());
  const auto tolerance = _pidController.GetSmartMotionAllowedClosedLoopError();
  if (error < tolerance) return 0.0;

  return _simSmartMotionProfile.Calculate(_smartMotionProfileTimer.Get())
      .velocity.value();
}