// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc/XboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/InstantCommand.h>
#include "Utilities/JoystickScaler.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();
  frc2::Command* GetAutonomousCommand();
  double ControllerGetLeftX();
  double ControllerGetLeftY();
  double ControllerGetRightX();

 private:
  // The robot's subsystems and commands are defined here...
  void ConfigureButtonBindings();
  JoystickScaler _controller {0, 2.5, 2.5};
  frc2::JoystickButton _rightBumber {&_controller, frc::XboxController::Button::kRightBumper};
  frc2::JoystickButton _leftBumber {&_controller, frc::XboxController::Button::kLeftBumper};
  frc2::JoystickButton _start {&_controller, frc::XboxController::Button::kStart};
};
