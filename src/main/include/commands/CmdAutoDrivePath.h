// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <pathplanner/lib/PathPlanner.h>
#include "subsystems/SubDriveBase.h"
#include <frc/Timer.h>


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class CmdAutoDrivePath
    : public frc2::CommandHelper<frc2::CommandBase, CmdAutoDrivePath> {
 public:
  CmdAutoDrivePath();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  frc::Timer timer;

  // This will load the file "Example Path.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2
  pathplanner::PathPlannerTrajectory _path = pathplanner::PathPlanner::loadPath("New Path", 8_mps, 5_mps_sq);
};
