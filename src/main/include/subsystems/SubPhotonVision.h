#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <units/length.h>
#include "utilities/ICSimVisionSystem.h" // copy paste of photonlib simvisionsystem, with a fix for 3d rotation
#include <photonlib/SimVisionTarget.h>
#include <optional>

class SubPhotonVision : public frc2::SubsystemBase
{
public:
  static SubPhotonVision &GetInstance()
  {
    static SubPhotonVision inst;
    return inst;
  }

  units::meter_t GetX();
  units::meter_t GetY();
  units::degree_t GetRot();
  std::optional <frc::Transform3d> GetBotToTarg();


  void Periodic() override;
  void SimulationPeriodic() override;

private:
  SubPhotonVision();
  
  // Camera data
  photonlib::PhotonCamera camera{"limelight"};
  const units::meter_t CAMERA_HEIGHT = 0.8_m;
  const units::radian_t CAMERA_PITCH = 15_deg;

  // Target data
  const units::meter_t TARGET_HEIGHT = 1_m;

  //simulate vision target
  frc::Transform2d _camtoRobot{{0_m,-0.2_m}, frc::Rotation2d {0_deg}};
  ic::SimVisionSystem _simVision{"limelight", 70_deg, _camtoRobot, 0.5_m, 5_m, 640, 480, 10};
  frc::Pose2d _target1Pose{54_ft, (27.0_ft/2) - 43.75_in - (48_in/2), 0_deg};
  photonlib::SimVisionTarget _visionTarget1{_target1Pose, 0.5_m, 34.6_in, 17_in};
};
