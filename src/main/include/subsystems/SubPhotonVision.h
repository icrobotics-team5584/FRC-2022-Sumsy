// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>

class SubPhotonVision : public frc2::SubsystemBase {
 public:
  SubPhotonVision();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
   const units::meter_t CAMERA_HEIGHT = 0.8_m;
  const units::meter_t TARGET_HEIGHT = 1_m;
  bool HasTarget = false;

  // Angle between horizontal and the camera.
  const units::radian_t CAMERA_PITCH = 15_deg;

  // How far from the target we want to be
  const units::meter_t GOAL_RANGE_METERS = 0.2_m;

  photonlib::PhotonCamera camera{"photonvision"};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
