#include "subsystems/SubElevator.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/InstantCommand.h>

SubElevator::SubElevator() {
    _elevator.SetPIDF(P, I, D, F);
    _elevator.SetPositionConversionFactor(ELEVATOR_POS_CONVERSION_FACTOR);
    _elevator.SetVelocityConversionFactor(ELEVATOR_VEL_CONVERSION_FACTOR);
}

void SubElevator::Periodic() {}

void SubElevator::DriveTo(units::meter_t height) {
    _elevator.SetPositionTarget(height.value());
}

void SubElevator::DriveAt(units::meters_per_second_t velocity) {
    _elevator.SetVelocityTarget(velocity.value());
}

void SubElevator::Stop() {
    _elevator.StopMotor();
}

void SubElevator::SimulationPeriodic() {
    _sim.SetInputVoltage(_elevator.GetSimVoltage());
    _sim.Update(20_ms);
    _elevator.GetEncoderRef().SetPosition(_sim.GetPosition().value());
    frc::SmartDashboard::PutNumber("elevator pos", _elevator.GetEncoderRef().GetPosition());   
}