#include "TankContainer.h"
#include "commands/CmdDeployPickup.h"
#include "commands/CmdPayloadOutake.h"
#include "subsystems/SubTankDrive.h"
#include "commands/CmdDriveRobotTank.h"

TankContainer::TankContainer(){
  // Initialize all of your commands and subsystems here
  SubTankDrive::GetInstance().SetDefaultCommand(CmdDriveRobotTank{&_controller});
  // Configure the button bindings
  ConfigureButtonBindings();
}

void TankContainer::ConfigureButtonBindings() {
  using BtnId = frc::XboxController::Button;
  using Btn = frc2::JoystickButton; 
  // Configure your button bindings here
  // Btn{&_controller, BtnId::buttonHere}.WhenPressed(commandHere{});
  Btn{&_controller, BtnId::kRightBumper}.WhileHeld(CmdDeployPickup{});
  Btn{&_controller, BtnId::kLeftBumper}.WhileHeld(CmdPayloadOutake{});
}

frc2::Command* TankContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return nullptr;
}
