// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdAutoDrivePath.h"


CmdAutoDrivePath::CmdAutoDrivePath() {
  AddRequirements(&SubDriveBase::GetInstance());
}

// Called when the command is initially scheduled.
void CmdAutoDrivePath::Initialize() {
  timer.Reset();
  timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void CmdAutoDrivePath::Execute() {
  // Sample the state of the path at 1.2 seconds
  pathplanner::PathPlannerTrajectory::PathPlannerState nextState = _path.sample(timer.Get()); 
  SubDriveBase::GetInstance().DriveToPathPoint(nextState.pose, nextState.velocity, nextState.holonomicRotation);


}

// Called once the command ends or is interrupted.
void CmdAutoDrivePath::End(bool interrupted) {}

// Returns true when the command should end.
bool CmdAutoDrivePath::IsFinished() {
  return false;
}
