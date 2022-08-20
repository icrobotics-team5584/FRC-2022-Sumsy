#include "subsystems/SubElevator.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>

SubElevator::SubElevator() {
    _elevator.SetPIDF(P, I, D);
    _elevator.GetEncoderRef().SetPositionConversionFactor(ELEVATOR_POS_CONVERSION_FACTOR);
    _elevator.GetEncoderRef().SetVelocityConversionFactor(ELEVATOR_VEL_CONVERSION_FACTOR);

    frc::SmartDashboard::PutNumber("Elevator/P", P);
    frc::SmartDashboard::PutNumber("Elevator/I", I);
    frc::SmartDashboard::PutNumber("Elevator/D", D);
}

void SubElevator::Periodic() {
    double newP = frc::SmartDashboard::GetNumber("Elevator/P", 0);
    double newI = frc::SmartDashboard::GetNumber("Elevator/I", 0);
    double newD = frc::SmartDashboard::GetNumber("Elevator/D", 0);
    if (newP != P || newI != I || newD != D) {
        P = newP;
        I = newI;
        D = newD;
        _elevator.SetPIDF(P,I,D);
    }
}

void SubElevator::DriveTo(units::meter_t height) {
    _elevator.SetTarget(height.value(), rev::ControlType::kSmartMotion);
}

void SubElevator::DriveAt(units::meters_per_second_t velocity) {
    double arbFF = _elevatorFF.Calculate(velocity).value();
    _elevator.SetTarget(velocity.value(), rev::ControlType::kVelocity, 0, arbFF);
}

void SubElevator::DriveWith(units::volt_t voltage) {
    _elevator.SetTarget(voltage.value(), rev::ControlType::kVoltage);
}

void SubElevator::DumbDriveTo(units::meter_t height) {
    _elevator.SetTarget(height.value(), rev::ControlType::kPosition);
}

void SubElevator::DumbDriveAt(double power) {
    _elevator.SetTarget(power, rev::ControlType::kDutyCycle);
}

void SubElevator::Stop() {
    _elevator.StopMotor();
}

void SubElevator::SimulationPeriodic() {
    auto simVoltage = _elevator.GetSimVoltage();
    _sim.SetInputVoltage(simVoltage);
    _sim.Update(20_ms);
    _elevator.UpdateSimEncoder(_sim.GetPosition().value());
    frc::SmartDashboard::PutNumber("Elevator/voltage", simVoltage.value());
    frc::SmartDashboard::PutNumber("Elevator/velocity", _elevator.GetEncoderRef().GetVelocity());
    frc::SmartDashboard::PutNumber("Elevator/pos", _elevator.GetEncoderRef().GetPosition());   
}