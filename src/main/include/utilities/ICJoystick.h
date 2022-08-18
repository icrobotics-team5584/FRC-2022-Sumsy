#pragma once

#include <frc/XboxController.h>
#include <frc2/command/button/JoystickButton.h>

class ICJoystick : frc::XboxController {
 private:
  using btn = frc::XboxController::Button;

 public:
  ICJoystick(int id = 0) : frc::XboxController(id){};

  frc2::JoystickButton A{this, btn::kA};
  frc2::JoystickButton B{this, btn::kB};
  frc2::JoystickButton Back{this, btn::kBack};
  frc2::JoystickButton LeftBumper{this, btn::kLeftBumper};
  frc2::JoystickButton LeftStick{this, btn::kLeftStick};
  frc2::JoystickButton RightBumper{this, btn::kRightBumper};
  frc2::JoystickButton RightStick{this, btn::kRightStick};
  frc2::JoystickButton Start{this, btn::kStart};
  frc2::JoystickButton X{this, btn::kX};
  frc2::JoystickButton Y{this, btn::kY};
};
