// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdIntake.h"

CmdIntake::CmdIntake(SubIntake* subIntake) {
  // Use addRequirements() here to declare subsystem dependencies.
  _subIntake = subIntake;
}

// Called when the command is initially scheduled.
void CmdIntake::Initialize() {

  _subIntake->Extend();
  _subIntake->Intake();
}

// Called repeatedly when this Command is scheduled to run
void CmdIntake::Execute() {}

// Called once the command ends or is interrupted.
void CmdIntake::End(bool interrupted) {
  _subIntake->Stop();
  _subIntake->Retract();
}

// Returns true when the command should end.
bool CmdIntake::IsFinished() {
  return false;
}
