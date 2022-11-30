// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdSnapToAngle.h"
#include "subsystems/SubDriveBase.h"

CmdSnapToAngle::CmdSnapToAngle(int targetAngle) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(&SubDriveBase::GetInstance());
  _targetAngle = targetAngle;
}

// Called when the command is initially scheduled.
void CmdSnapToAngle::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CmdSnapToAngle::Execute() {
   SubDriveBase::GetInstance().SnapToAngle(_targetAngle * 1_deg);
}

// Called once the command ends or is interrupted.
void CmdSnapToAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool CmdSnapToAngle::IsFinished() {
  return false;
}
