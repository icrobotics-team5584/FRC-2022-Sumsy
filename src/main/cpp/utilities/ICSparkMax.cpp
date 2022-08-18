#include "utilities/ICSparkMax.h"

#include <frc/RobotBase.h>

ICSparkMax::ICSparkMax(int deviceID, Type type)
    : rev::CANSparkMax(deviceID,
                       rev::CANSparkMaxLowLevel::MotorType::kBrushless) {
    _type = type;
    RestoreFactoryDefaults();
    SetSmartCurrentLimit(type == Type::NEO ? NEO_CURRENT_LIMIT
                                           : NEO_550_CURRENT_LIMIT);
};

void ICSparkMax::SetPIDF(double P, double I, double D, double F) {
    _pidController.SetP(P);
    _pidController.SetI(I);
    _pidController.SetD(D);
    _pidController.SetFF(F);

    // _simSmartMotionController.SetPID(P, I, D);
    _simVelocityController.SetPID(P, I, D);
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
            return units::volt_t{_simVelocityController.Calculate(_encoder.GetPosition(), _target)};

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

            /*
            const auto measurement = units::turn_t{_encoder.GetPosition()};
            const auto goal = units::turn_t{_target};
            return units::volt_t{_simSmartMotionController.Calculate(measurement, goal)};
            */
        case rev::ControlType::kCurrent:
            break;

        case rev::ControlType::kSmartVelocity:
            break;

    }
    return units::volt_t{Get()};
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


void ICSparkMax::StopMotor() {
    _controlType = rev::ControlType::kDutyCycle;
    _target = 0;
    CANSparkMax::StopMotor();
}