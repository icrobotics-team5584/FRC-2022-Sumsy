// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/CmdDriveRobot.h"
#include "subsystems/SubDriveBase.h"

CmdDriveRobot::CmdDriveRobot(frc::XboxController* controller) {
  // Use addRequirements() here to declare subsystem dependencies.
  _controller = controller;
  AddRequirements(&SubDriveBase::GetInstance());
}

// Called when the command is initially scheduled.
void CmdDriveRobot::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void CmdDriveRobot::Execute() {
  const double deadband = 0.17;

  // Get the x speed. We are inverting this because Xbox controllers return
  // negative values when we push forward.
  const auto xSpeed = -m_xspeedLimiter.Calculate(
                          frc::ApplyDeadband(_controller -> GetLeftY(), deadband)) *
                      SubDriveBase::MAX_VELOCITY;
  // Get the y speed or sideways/strafe speed. We are inverting this because
  // we want a positive value when we pull to the left. Xbox controllers
  // return positive values when you pull to the right by default.
  const auto ySpeed = -m_yspeedLimiter.Calculate(
                          frc::ApplyDeadband(_controller -> GetLeftX(), deadband)) *
                      SubDriveBase::MAX_VELOCITY;
  // Get the rate of angular rotation. We are inverting this because we want a
  // positive value when we pull to the left (remember, CCW is positive in
  // mathematics). Xbox controllers return positive values when you pull to
  // the right by default.
  const auto rot = -m_rotLimiter.Calculate(
                       frc::ApplyDeadband(_controller -> GetRightX(), deadband)) *
                   SubDriveBase::MAX_ANGULAR_VELOCITY;
  SubDriveBase::GetInstance().Drive(xSpeed, ySpeed, rot, true);
}

// Called once the command ends or is interrupted.
void CmdDriveRobot::End(bool interrupted) {}

// Returns true when the command should end.
bool CmdDriveRobot::IsFinished() {
  return false;
}
