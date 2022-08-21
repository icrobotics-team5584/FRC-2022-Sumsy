#include "utilities/ICSparkMax.h"

#include <frc/RobotBase.h>
#include <units/voltage.h>
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
};

void ICSparkMax::SetTarget(double target, rev::ControlType controlType,
                           int pidSlot, double arbFeedForward) {
    _target = target;
    _pidSlot = pidSlot;
    _arbFeedForward = arbFeedForward;
    SetInternalControlType(controlType);
    _pidController.SetReference(target, controlType, pidSlot, _arbFeedForward);

    SyncSimPID();
}

double ICSparkMax::GetTarget() {
    if (frc::RobotBase::IsSimulation() && _controlType == rev::ControlType::kSmartMotion) {
        return _simSmartMotionProfile.Calculate(_timeSinceSmartMotionStart).position.value();
    }
    return _target;
}

void ICSparkMax::Set(double speed) {
    if (frc::RobotBase::IsSimulation()) {
        SetTarget(speed, rev::ControlType::kDutyCycle);
    } 
    CANSparkMax::Set(speed);
}

void ICSparkMax::SetVoltage(units::volt_t output) {
    SetTarget(output.value(), rev::ControlType::kVoltage);
}

units::volt_t ICSparkMax::GetSimVoltage() {
    auto targState = _simSmartMotionProfile.Calculate(_timeSinceSmartMotionStart);
    units::volt_t output = 0_V;

    switch (_controlType) {
        case rev::ControlType::kDutyCycle:
            output = units::volt_t{_target * 12};
            break;

        case rev::ControlType::kVelocity:
            output = units::volt_t { 
                _simController.Calculate(_encoder.GetVelocity(),_target) 
                + _pidController.GetFF() + _arbFeedForward
            };
            break;

        case rev::ControlType::kPosition:
            output = units::volt_t { 
                _simController.Calculate(_encoder.GetPosition(),_target) 
                + _pidController.GetFF() + _arbFeedForward
            };
            break;

        case rev::ControlType::kVoltage:
            output = units::volt_t{_target};
            break;

        case rev::ControlType::kSmartMotion:
            output = units::volt_t{
                _simController.Calculate(
                    _encoder.GetPosition(), 
                    _simSmartMotionProfile.Calculate(_timeSinceSmartMotionStart)
                        .position
                        .value()
                ) + _pidController.GetFF() + _arbFeedForward
            };
            break;

        case rev::ControlType::kCurrent:
            std::cout << "Warning: closed loop Current control not supported by ICSparkMax in Simulation\n";
            break;

        case rev::ControlType::kSmartVelocity:
            std::cout << "Warning: closed loop Smart Velocity control not supported by ICSparkMax in Simulation\n";
            break;
    }
    return std::clamp(output, -12_V, 12_V);
}

void ICSparkMax::StopMotor() {
    _target = 0;
    SetInternalControlType(rev::ControlType::kDutyCycle);
    CANSparkMax::StopMotor();
}

void ICSparkMax::SetInternalControlType(rev::ControlType controlType) {
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
    _simController.SetIntegratorRange(-_pidController.GetIMaxAccum(), _pidController.GetIMaxAccum());

    if (_controlType == rev::ControlType::kSmartMotion) {
        frc::TrapezoidProfile<units::meters>::Constraints constrainsts = {
            units::meters_per_second_t{_pidController.GetSmartMotionMaxVelocity()},
            units::meters_per_second_squared_t{_pidController.GetSmartMotionMaxAccel()}
        };

        _simSmartMotionProfile = {
            constrainsts,
            {units::meter_t{_target}, 0_mps},
            {units::meter_t{_encoder.GetPosition()}, units::meters_per_second_t{_encoder.GetVelocity()}} 
        };

        _smartMotionProfileNotifier.StartPeriodic(20_ms);

    } else {
        _smartMotionProfileNotifier.Stop();
    }
    _timeSinceSmartMotionStart = 0_ms;

}