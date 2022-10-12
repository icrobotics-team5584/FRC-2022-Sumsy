// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubPhotonVision.h"
#include <photonlib/PhotonUtils.h>
#include <frc/smartdashboard/SmartDashboard.h>

SubPhotonVision::SubPhotonVision() = default;

// This method will be called once per scheduler run
void SubPhotonVision::Periodic() {

    photonlib::PhotonPipelineResult result = camera.GetLatestResult();
    

    if (result.HasTargets()) {
      //First calculate range
      auto bestTarget = result.GetBestTarget();
      HasTarget = true;
      frc::SmartDashboard::PutBoolean("PhotonVisionTargetFound", HasTarget);
     /* units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(CAMERA_HEIGHT, TARGET_HEIGHT, CAMERA_PITCH,units::degree_t{result.GetBestTarget().GetPitch()});
      frc::SmartDashboard::PutNumber("PhotonVisionRangeToTarget", double(range));
      frc::SmartDashboard::PutNumber("LimeLightYaw", result.GetBestTarget().GetYaw()); */
      frc::SmartDashboard::PutNumber("target Distance", bestTarget.GetCameraRelativePose().X().value()); 
      frc::SmartDashboard::PutNumber("target Rotation", bestTarget.GetCameraRelativePose().Rotation().Degrees().value()); 
      frc::SmartDashboard::PutNumber("target Y", bestTarget.GetCameraRelativePose().Y().value()); 

      // Use this range as the measurement we give to the PID controller.
      //forwardSpeed =-controller.Calculate(range.value(), GOAL_RANGE_METERS.value());
          
    } else {
      // If we have no targets, stay still.
      //forwardSpeed = 0;
      HasTarget = false;
       frc::SmartDashboard::PutBoolean("PhotonVisionTargetFound", HasTarget);
    }

}

units::meter_t SubPhotonVision::GetX() {
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  
    if (result.HasTargets()) {
      auto bestTarget = result.GetBestTarget();
      return bestTarget.GetCameraRelativePose().X();
    } else {
      return 0_m;
    }
}