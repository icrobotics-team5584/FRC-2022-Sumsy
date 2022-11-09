// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdDriveRobotTank.h"
#include "subsystems/SubDriveBase.h"

CmdDriveRobotTank::CmdDriveRobotTank(frc::XboxController* controller) {
  // Use addRequirements() here to declare subsystem dependencies.

  _controller = controller;
  AddRequirements(&SubDriveBase::GetInstance());
}

// Called when the command is initially scheduled.
void CmdDriveRobotTank::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CmdDriveRobotTank::Execute() {

}

// Called once the command ends or is interrupted.
void CmdDriveRobotTank::End(bool interrupted) {}

// Returns true when the command should end.
bool CmdDriveRobotTank::IsFinished() {
  return false;
}
