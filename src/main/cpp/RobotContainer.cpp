// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "commands/CmdDeployPickup.h"

RobotContainer::RobotContainer(){
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  _rightBumber.WhileHeld(CmdDeployPickup{});
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}

double RobotContainer::ControllerGetLeftX() {
  return _controller.GetLeftX();
}

double RobotContainer::ControllerGetLeftY() {
  return _controller.GetLeftY();
}

double RobotContainer::ControllerGetRightX() {
  return _controller.GetRightX();
}