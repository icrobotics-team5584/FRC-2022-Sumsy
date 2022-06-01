// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <wpi/numbers>

#include "utilities/SwerveModule.h"

class SubDriveBase : public frc2::SubsystemBase {
 public:
  SubDriveBase();

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);
             
  void UpdateOdometry();

  void ZeroSensors();

  static constexpr units::meters_per_second_t kMaxSpeed =
      3.0_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      wpi::numbers::pi};

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  frc::Translation2d m_frontLeftLocation{+0.281_m, +0.281_m};
  frc::Translation2d m_frontRightLocation{+0.281_m, -0.281_m};
  frc::Translation2d m_backLeftLocation{-0.281_m, +0.281_m};
  frc::Translation2d m_backRightLocation{-0.281_m, -0.281_m};

  SwerveModule m_frontLeft{1, 2, 11};
  SwerveModule m_frontRight{3, 4, 9};
  SwerveModule m_backLeft{5, 6, 12};
  SwerveModule m_backRight{7, 8, 10};

  frc::AnalogGyro m_gyro{0};

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, m_gyro.GetRotation2d()};
};

