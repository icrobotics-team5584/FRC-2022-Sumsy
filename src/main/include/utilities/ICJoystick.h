#pragma once

#include <frc/XboxController.h>
#include <frc2/command/button/JoystickButton.h>

class ICJoystick {
 public:
  ICJoystick(int id = 0) : _controller(id){};
  frc::XboxController& Get() {return _controller;};

  frc::XboxController _controller;

  frc2::JoystickButton A{&_controller, btn::kA};
  frc2::JoystickButton B{&_controller, btn::kB};
  frc2::JoystickButton Back{&_controller, btn::kBack};
  frc2::JoystickButton LeftBumper{&_controller, btn::kLeftBumper};
  frc2::JoystickButton LeftStick{&_controller, btn::kLeftStick};
  frc2::JoystickButton RightBumper{&_controller, btn::kRightBumper};
  frc2::JoystickButton RightStick{&_controller, btn::kRightStick};
  frc2::JoystickButton Start{&_controller, btn::kStart};
  frc2::JoystickButton X{&_controller, btn::kX};
  frc2::JoystickButton Y{&_controller, btn::kY};

 private:
  typedef frc::XboxController::Button btn;
};
