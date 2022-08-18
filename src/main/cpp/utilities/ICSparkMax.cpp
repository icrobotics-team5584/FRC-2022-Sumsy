#include "utilities/ICSparkMax.h"

#include <frc/RobotBase.h>

ICSparkMax::ICSparkMax(int deviceID, Type type)
    : rev::CANSparkMax(deviceID,
                       rev::CANSparkMaxLowLevel::MotorType::kBrushless),
      _simDeviceSim("SPARK MAX", deviceID) {
    _type = type;
    RestoreFactoryDefaults();
    SetSmartCurrentLimit(type == Type::NEO ? NEO_CURRENT_LIMIT
                                           : NEO_550_CURRENT_LIMIT);

    if (frc::RobotBase::IsSimulation()) {
        _simNotifier.StartPeriodic(20_ms);
    }
};

void ICSparkMax::SetPIDF(double P, double I, double D, double F) {
    _pidController.SetP(P);
    _pidController.SetI(I);
    _pidController.SetD(D);
    _pidController.SetFF(F);
}

void ICSparkMax::SetPositionTarget(double target) {
    _target = target;
    _controlType = rev::ControlType::kSmartMotion;
    _pidController.SetReference(target, rev::ControlType::kSmartMotion);
}

void ICSparkMax::SetVelocityTarget(double target) {
    _target = target;
    _controlType = rev::ControlType::kVelocity;
    _pidController.SetReference(target, rev::ControlType::kVelocity);
}

void ICSparkMax::SetPositionConversionFactor(double factor) {
    _encoder.SetPositionConversionFactor(factor);
}

void ICSparkMax::SetVelocityConversionFactor(double factor) {
    _encoder.SetVelocityConversionFactor(factor);
}

units::volt_t ICSparkMax::GetSimVoltage() {
    switch (_controlType) {
        case rev::ControlType::kDutyCycle:
            return units::volt_t{Get()};
        case rev::ControlType::kVelocity:
            return units::volt_t{(_target * _encoder.GetVelocityConversionFactor() / 5676.0) * 12};
        case rev::ControlType::kVoltage:
            return units::volt_t{_target};
        case rev::ControlType::kPosition:
            break;
        case rev::ControlType::kSmartMotion:
            if (GoingForward()) {
                return 12_V;
            } else if (GoingBackward()) {
                return -12_V;
            } 
            return 0_V;
        case rev::ControlType::kCurrent:
            break;
        case rev::ControlType::kSmartVelocity:
            break;
        default:
            return units::volt_t{Get()};
    }
}

bool ICSparkMax::GoingForward() {
    if (_controlType == rev::ControlType::kSmartMotion) {
        return _encoder.GetPosition() < _target;
    } else if (_controlType == rev::ControlType::kVelocity) {
        return _target > 0;
    } else {
        return Get() > 0;
    }
}

bool ICSparkMax::GoingBackward() {
    if (_controlType == rev::ControlType::kSmartMotion) {
        return _encoder.GetPosition() > _target;
    } else if (_controlType == rev::ControlType::kVelocity) {
        return _target < 0;
    } else {
        return Get() < 0;
    }
}
