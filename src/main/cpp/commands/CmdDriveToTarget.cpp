// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdDriveToTarget.h"
#include "subsystems/SubPhotonVision.h"
#include "subsystems/SubDriveBase.h"
#include <units/length.h>

CmdDriveToTarget::CmdDriveToTarget() {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(&SubDriveBase::GetInstance());
}

// Called when the command is initially scheduled.
void CmdDriveToTarget::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CmdDriveToTarget::Execute() {
  auto distanceX = SubPhotonVision::GetInstance().GetX();
  auto distanceY = SubPhotonVision::GetInstance().GetY();
  SubDriveBase::GetInstance().DriveToTarget(distanceX, distanceY, 0.5_m);
}

// Called once the command ends or is interrupted.
void CmdDriveToTarget::End(bool interrupted) {}

// Returns true when the command should end.
bool CmdDriveToTarget::IsFinished() {
  return false;
}
