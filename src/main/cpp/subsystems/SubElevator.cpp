// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubElevator.h"
#include <frc/smartdashboard/SmartDashboard.h>

SubElevator::SubElevator() {
    _spmLeftElevator.SetPIDFF(kP, kI, kD, kFF);
    _spmLeftElevator.SetConversionFactors(
        POS_CONVERSION_FACTOR, VEL_CONVERSION_FACTOR, VEL_CONVERSION_FACTOR);

    _spmLeftElevator.SetSmartMotionMaxAccel(MAX_ACCERLATION.value());
    _spmLeftElevator.SetSmartMotionMaxVelocity(MAX_VELOCITY.value());

    _spmRightElevator.Follow(_spmLeftElevator);
    _spmRightElevator.SetSmartCurrentLimit(40);
    _spmLeftElevator.SetSmartCurrentLimit(40);
    frc::SmartDashboard::PutData("Left Elevator Motor",(wpi::Sendable*)&_spmLeftElevator);
}


// This method will be called once per scheduler run
void SubElevator::Periodic() {
frc::SmartDashboard::PutNumber("Elevator Target Value", _spmLeftElevator.GetTarget());
frc::SmartDashboard::PutNumber("Elevator Current Position", _spmLeftElevator.GetPosition());



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


// Don't let the Elevator kill itself
  if ((AtLowerLimit() && GoingDown())) {
    frc::SmartDashboard::PutBoolean("Elevator Lower Safety", true);
    _targetPosition = FIRST_POSITION();
    _spmLeftElevator.SetTarget(_targetPosition, rev::CANSparkMax::ControlType::kSmartMotion);
    _spmLeftElevator.Set(0);
  } else {
    frc::SmartDashboard::PutBoolean("Elevator LowerLeft Safety", false);
  }
  if ((AtLowerLimit() && GoingDown())) {
    _targetPosition = FIRST_POSITION();
    _spmRightElevator.SetTarget(_targetPosition, rev::CANSparkMax::ControlType::kSmartMotion);
    _spmRightElevator.Set(0);
    frc::SmartDashboard::PutBoolean("Elevator LowerRight Safety", true);
  } else {
    frc::SmartDashboard::PutBoolean("Elevator LowerRight Safety", false);
  }

  // Don't let the Elevator kill itself pt.2
  if ((AtUpperLimit() && GoingUp())) {
    frc::SmartDashboard::PutBoolean("Elevator Upper Safety", true);
    _targetPosition = THIRD_POSITION();
    _spmLeftElevator.SetTarget(_targetPosition, rev::CANSparkMax::ControlType::kSmartMotion);
    _spmLeftElevator.Set(0);
  } else {
    frc::SmartDashboard::PutBoolean("Elevator UpperLeft Safety", false);
  }
  if ((AtUpperLimit() && GoingUp())) {
    _targetPosition = THIRD_POSITION();
    _spmRightElevator.SetTarget(_targetPosition, rev::CANSparkMax::ControlType::kSmartMotion);
    _spmRightElevator.Set(0);
    frc::SmartDashboard::PutBoolean("Elevator UpperRight Safety", true);
  } else {
    frc::SmartDashboard::PutBoolean("Elevator UpperRight Safety", false);
  }


 }
bool SubElevator::AtLowerLimit() { return !_Lowerlmt.Get(); }
bool SubElevator::AtUpperLimit() { return !_Upperlmt.Get(); }

bool SubElevator::GoingDown() {
  if (_inSmartMotionMode) {
    return _spmLeftElevator.GetPosition() > _targetPosition ||
           _spmLeftElevator.GetPosition() > _targetPosition;
  } else {
    return _spmRightElevator.Get() < 0 || _spmRightElevator.Get() < 0;
  }
  
}

bool SubElevator::GoingUp() {
  if (_inSmartMotionMode) {
    return _spmLeftElevator.GetPosition() < _targetPosition ||
           _spmRightElevator.GetPosition() < _targetPosition;
  } else {
    return _spmRightElevator.Get() > 0 || _spmRightElevator.Get() > 0;
  }
  
}

