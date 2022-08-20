#include "subsystems/SubElevator.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>

SubElevator::SubElevator() {
    _elevator.SetPIDF(P, I, D, F);
    _elevator.GetEncoderRef().SetPositionConversionFactor(ELEVATOR_POS_CONVERSION_FACTOR);
    _elevator.GetEncoderRef().SetVelocityConversionFactor(ELEVATOR_VEL_CONVERSION_FACTOR);
}

void SubElevator::Periodic() {}

void SubElevator::DriveTo(units::meter_t height) {
    _elevator.SetTarget(height.value(), rev::ControlType::kSmartMotion);
}

void SubElevator::DriveAt(units::meters_per_second_t velocity) {
    _elevator.SetTarget(velocity.value(), rev::ControlType::kVelocity);
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