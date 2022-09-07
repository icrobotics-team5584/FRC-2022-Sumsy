// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/CmdDeployPickup.h"

#include <frc2/command/button/JoystickButton.h>
#include "commands/CmdElevatorSequence.h"
#include "subsystems/SubElevator.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
 SubElevator::Getinstance();
  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  using  BtnId = frc::XboxController::Button;
  using Btn = frc2::JoystickButton; 
  
  Btn{&_controller, BtnId::kA}.WhenPressed(CmdElevatorSequence{});
  // Configure your button bindings here
  _rightBumber.WhileHeld(CmdDeployPickup{});
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
