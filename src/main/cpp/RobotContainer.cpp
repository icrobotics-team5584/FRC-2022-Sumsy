// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/CmdDeployPickup.h"

#include <frc2/command/button/JoystickButton.h>

RobotContainer::RobotContainer() {
  using btn = frc2::JoystickButton;
  using xboxbtn = frc::XboxController::Button;

  btn{&_controller, xboxbtn::kLeftBumper}.WhileHeld(
      [=] { SubElevator::Getinstance().Extendfirst(); });

  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  using  Btn = frc::XboxController::Button;
  
  frc2::JoystickButton{&_controller, frc::XboxController::Button::kA};
  // Configure your button bindings here
  _rightBumber.WhileHeld(CmdDeployPickup{});
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
