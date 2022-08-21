#include "subsystems/SubElevator.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>

SubElevator::SubElevator() {
    _elevator.GetPIDControllerRef().SetP(P);
    _elevator.GetPIDControllerRef().SetI(I);
    _elevator.GetPIDControllerRef().SetD(D);
    _elevator.GetEncoderRef().SetPositionConversionFactor(ELEVATOR_POS_CONVERSION_FACTOR);
    _elevator.GetEncoderRef().SetVelocityConversionFactor(ELEVATOR_VEL_CONVERSION_FACTOR);
    _elevator.GetPIDControllerRef().SetSmartMotionMaxAccel(maxAcceleration.value());
    _elevator.GetPIDControllerRef().SetSmartMotionMaxVelocity(maxVelocity.value());
    _elevator.GetPIDControllerRef().SetSmartMotionAllowedClosedLoopError(tolerance.value());

    frc::SmartDashboard::PutData("elevator spark", (wpi::Sendable*)&_elevator);
}

void SubElevator::Periodic() {}

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

void SubElevator::Stop() {
    _elevator.Set(0);
}

void SubElevator::SimulationPeriodic() {
    _sim.SetInputVoltage(_elevator.GetSimVoltage());
    _sim.Update(20_ms);
    _elevator.UpdateSimEncoder(_sim.GetPosition().value(), _sim.GetVelocity().value());
}