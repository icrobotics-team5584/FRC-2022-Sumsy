#include "subsystems/SubPhotonVision.h"
#include <photonlib/PhotonUtils.h>
#include <frc/smartdashboard/SmartDashboard.h>

SubPhotonVision::SubPhotonVision() = default;

// This method will be called once per scheduler run
void SubPhotonVision::Periodic() {
    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    
    if (result.HasTargets()) {
      auto bestTarget = result.GetBestTarget();
      auto botToTarg = bestTarget.GetCameraToTarget();

      frc::SmartDashboard::PutNumber("target X", botToTarg.X().value());
      frc::SmartDashboard::PutNumber("target Rot",
                                     botToTarg.Rotation().Z().value());
      frc::SmartDashboard::PutNumber("target Y", botToTarg.Y().value());
    }
}

units::meter_t SubPhotonVision::GetX() {
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  
    if (result.HasTargets()) {
      auto bestTarget = result.GetBestTarget();
      return bestTarget.GetCameraToTarget().X();
    } else {
      return 0_m;
    }
}
units::meter_t SubPhotonVision::GetY() {
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  
    if (result.HasTargets()) {
      auto bestTarget = result.GetBestTarget();
      return bestTarget.GetCameraToTarget().Y();
    } else {
      return 0_m;
    }
}