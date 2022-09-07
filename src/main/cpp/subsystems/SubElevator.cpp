// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubElevator.h"
#include <frc/smartdashboard/SmartDashboard.h>

SubElevator::SubElevator() {
    _pidLeftMotorController.SetP(kP);
    _pidLeftMotorController.SetI(kI);
    _pidLeftMotorController.SetD(kD);
    _pidLeftMotorController.SetSmartMotionMaxAccel(MAX_ACCERLATION.value());
    _pidLeftMotorController.SetSmartMotionMaxVelocity(MAX_VELOCITY.value());

    _spmLeftElevator.GetEncoderRef().SetPositionConversionFactor(CONVERSTION_FACTOR);

    _spmRightElevator.Follow(_spmLeftElevator);
    _spmRightElevator.SetSmartCurrentLimit(40);
    _spmLeftElevator.SetSmartCurrentLimit(40);
    frc::SmartDashboard::PutData("Left Elevator Motor",(wpi::Sendable*)&_spmLeftElevator);
}


// This method will be called once per scheduler run
void SubElevator::Periodic() {
frc::SmartDashboard::PutNumber("Elevator Target Value", _spmLeftElevator.GetTarget());
frc::SmartDashboard::PutNumber("Elevator Current Position", _spmLeftElevator.GetEncoderRef().GetPosition());



}


void SubElevator::Extendfirst() {   
  _spmLeftElevator.SetTarget(FIRST_POSITION.value(), rev::CANSparkMax::ControlType::kSmartMotion);

}

void SubElevator::SimulationPeriodic() { 

  //calculate voltage going into the elevator

  units::volt_t applyVoltage =_spmLeftElevator.GetSimVoltage(); //calculates the voltage

  //Update physics simulation with the volatge we gave it

  _elevatorSim.SetInputVoltage(applyVoltage); //applies the voltage to simulator
  _elevatorSim.Update(20_ms); //updates physics simulation every 20ms

  //Updates our encoder position to simulator position

  auto verticleVelocity = _elevatorSim.GetVelocity(); //elevator sim gets the velocity 
  units::meter_t verticleDistence = _elevatorSim.GetPosition();//elevator sim gets the verticle distence
  _spmLeftElevator.UpdateSimEncoder(verticleDistence.value(),verticleVelocity.value());//position of the encoder is the verticle distence
}
