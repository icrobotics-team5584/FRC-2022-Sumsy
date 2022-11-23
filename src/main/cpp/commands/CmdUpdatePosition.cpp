// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdUpdatePosition.h"
#include "subsystems/SubDriveBase.h"
#include "subsystems/SubPhotonVision.h"
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform3d.h>
#include <units/length.h>


CmdUpdatePosition::CmdUpdatePosition() {
 AddRequirements(&SubPhotonVision::GetInstance());
}

// Called when the command is initially scheduled.
void CmdUpdatePosition::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CmdUpdatePosition::Execute() {
auto botToTarget = SubPhotonVision::GetInstance().GetBotToTarg();
if (botToTarget.has_value()) {
frc::Pose3d fieldTarget = frc::Pose3d{1_m, 1_m, 0.5_m, {}};
auto robotPosition = fieldTarget.TransformBy(botToTarget.value());
}
}

// Called once the command ends or is interrupted.
void CmdUpdatePosition::End(bool interrupted) {}

// Returns true when the command should end.
bool CmdUpdatePosition::IsFinished() {
  return false;
}
