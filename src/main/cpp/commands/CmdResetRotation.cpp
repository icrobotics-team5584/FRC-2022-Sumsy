// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdResetRotation.h"
#include "subsystems/SubDriveBase.h"

CmdResetRotation::CmdResetRotation() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void CmdResetRotation::Initialize() {
  SubDriveBase::GetInstance().ResetGyroHeading();
}

// Called repeatedly when this Command is scheduled to run
void CmdResetRotation::Execute() {}

// Called once the command ends or is interrupted.
void CmdResetRotation::End(bool interrupted) {}

// Returns true when the command should end.
bool CmdResetRotation::IsFinished() {
  return false;
}
