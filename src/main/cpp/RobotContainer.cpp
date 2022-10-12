// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "commands/CmdDeployPickup.h"
#include "commands/CmdPayloadOutake.h"
#include "commands/CmdResetRotation.h"
#include "subsystems/SubDriveBase.h"
#include "commands/CmdDriveRobot.h"

RobotContainer::RobotContainer(){
  // Initialize all of your commands and subsystems here
  SubDriveBase::GetInstance().SetDefaultCommand(CmdDriveRobot{&_controller});
  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  using BtnId = frc::XboxController::Button;
  using Btn = frc2::JoystickButton; 

  // Configure your button bindings here
  // Btn{&_controller, BtnId::buttonHere}.WhenPressed(commandHere{});
  _rightBumber.WhileHeld(CmdDeployPickup{});
  _leftBumber.WhileHeld(CmdPayloadOutake{});
  _start.WhenPressed(CmdResetRotation{});
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