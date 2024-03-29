#include "subsystems/SubPhotonVision.h"
#include <photonlib/PhotonUtils.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SubDriveBase.h"
#include "units/angle.h"

SubPhotonVision::SubPhotonVision(){
  _simVision.AddSimVisionTarget(_visionTarget1);

  SubDriveBase::GetInstance().DisplayPose("visionTarget", _target1Pose);
}

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
      frc::SmartDashboard::PutNumber("Rotation", GetRot().value());
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
units::degree_t SubPhotonVision::GetRot() {
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  
    if (result.HasTargets()) {
      auto bestTarget = result.GetBestTarget();
      double yaw = bestTarget.GetYaw();
      units::degree_t rotation {yaw};
      return rotation;
      
    } else {
      return 0_deg;
    }
}

std::optional<frc::Transform3d> SubPhotonVision::GetBotToTarg() {
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();

  if (result.HasTargets()) {
    auto bestTarget = result.GetBestTarget();
    return bestTarget.GetCameraToTarget();
  } else {
    return {};
  }
}

void SubPhotonVision::SimulationPeriodic() {
  _simVision.ProcessFrame(SubDriveBase::GetInstance().GetPose());
}