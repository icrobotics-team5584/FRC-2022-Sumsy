// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <subsystems/SubElevator.h>

RobotContainer::RobotContainer(){
  // Initialize all of your commands and subsystems here
  SubElevator::GetInstance();

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  _controller.A.WhenPressed([=]{SubElevator::GetInstance().DumbDriveTo(1.4_m);});
  _controller.B.WhenPressed([=]{SubElevator::GetInstance().DumbDriveTo(0_m);});
  _controller.X.WhenPressed([=]{SubElevator::GetInstance().DriveAt(0.5_mps);});
  _controller.Y.WhenPressed([=]{SubElevator::GetInstance().DriveWith(6_V);});
  _controller.LeftBumper.WhenPressed([=]{SubElevator::GetInstance().DumbDriveTo(1.4_m);});
  _controller.RightBumper.WhenPressed([=]{SubElevator::GetInstance().DumbDriveAt(0.7);});
  _controller.Back.WhenPressed([=]{SubElevator::GetInstance().Stop();});
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
