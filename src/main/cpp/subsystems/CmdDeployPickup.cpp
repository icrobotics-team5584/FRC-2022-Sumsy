// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/CmdDeployPickup.h"
#include "subsystems/SubPickup.h"
#include "subsystems/SubPayload.h"

CmdDeployPickup::CmdDeployPickup() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void CmdDeployPickup::Initialize() {
  SubPickup::GetInstance().Extender();
  SubPickup::GetInstance().Intake();
  SubPayload::GetInstance().Intake();
}

// Called repeatedly when this Command is scheduled to run
void CmdDeployPickup::Execute() {}

// Called once the command ends or is interrupted.
void CmdDeployPickup::End(bool interrupted) {
  SubPickup::GetInstance().Retractor();
  SubPickup::GetInstance().Stoptake();
  SubPayload::GetInstance().Stop();
}

// Returns true when the command should end.
bool CmdDeployPickup::IsFinished() {
  if (SubPayload::GetInstance().HasBall()){
    return true;
  }
  else {
    return false;
  }
  
}
