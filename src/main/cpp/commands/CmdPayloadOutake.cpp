// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdPayloadOutake.h"
#include "subsystems/SubPayload.h"

CmdPayloadOutake::CmdPayloadOutake() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void CmdPayloadOutake::Initialize() {
  SubPayload::GetInstance().Outake();
}

// Called repeatedly when this Command is scheduled to run
void CmdPayloadOutake::Execute() {}

// Called once the command ends or is interrupted.
void CmdPayloadOutake::End(bool interrupted) {
  SubPayload::GetInstance().Stop();
}

// Returns true when the command should end.
bool CmdPayloadOutake::IsFinished() {
  return false;
}
