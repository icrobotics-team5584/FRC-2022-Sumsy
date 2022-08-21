// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <subsystems/SubElevator.h>
#include <frc2/command/button/JoystickButton.h>

RobotContainer::RobotContainer(){
  // Initialize all of your commands and subsystems here
  SubElevator::GetInstance();

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  using btn = frc2::JoystickButton;
  using btnId = frc::XboxController::Button;
  using AxsId = frc::XboxController::Axis;

  btn{&_controller, btnId::kA}.WhenPressed([=]{SubElevator::GetInstance().DriveTo(1.4_m);});
  btn{&_controller, btnId::kB}.WhenPressed([=]{SubElevator::GetInstance().DriveTo(0_m);});
  btn{&_controller, btnId::kX}.WhenPressed([=]{SubElevator::GetInstance().DriveWith(12_V);});
  btn{&_controller, btnId::kY}.WhenPressed([=]{SubElevator::GetInstance().DriveWith(-12_V);});
  btn{&_controller, btnId::kLeftBumper}.WhenPressed([=]{SubElevator::GetInstance().Stop();});
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
