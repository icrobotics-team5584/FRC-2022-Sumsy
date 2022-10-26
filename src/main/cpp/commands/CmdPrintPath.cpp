// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdPrintPath.h"
#include <pathplanner/lib/PathPlanner.h>
#include <iostream>

CmdPrintPath::CmdPrintPath() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void CmdPrintPath::Initialize() {
  // This will load the file "Example Path.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2
  pathplanner::PathPlannerTrajectory examplePath = pathplanner::PathPlanner::loadPath("New Path", 8_mps, 5_mps_sq);

// Sample the state of the path at 1.2 seconds
  pathplanner::PathPlannerTrajectory::PathPlannerState exampleState = examplePath.sample(1.2_s);

  std::cout << "X: " << exampleState.pose.X().value() << " Y: " << exampleState.pose.Y().value() << std::endl; 
  
}

// Called repeatedly when this Command is scheduled to run
void CmdPrintPath::Execute() {}

// Called once the command ends or is interrupted.
void CmdPrintPath::End(bool interrupted) {}

// Returns true when the command should end.
bool CmdPrintPath::IsFinished() {
  return false;
}
