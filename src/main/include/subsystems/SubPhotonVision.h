#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <units/length.h>

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

  void Periodic() override;

private:
  SubPhotonVision();
  
  // Camera data
  photonlib::PhotonCamera camera{"limelight"};
  const units::meter_t CAMERA_HEIGHT = 0.8_m;
  const units::radian_t CAMERA_PITCH = 15_deg;

  // Target data
  const units::meter_t TARGET_HEIGHT = 1_m;
};
