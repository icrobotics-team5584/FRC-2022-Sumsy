// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AnalogGyro.h>
#include <AHRS.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <wpi/numbers>
#include <frc/smartdashboard/Field2d.h>

#include "Constants.h"
#include "utilities/SwerveModule.h"

class SubDriveBase : public frc2::SubsystemBase {
 public:
  void Periodic() override;

  static SubDriveBase &GetInstance() {static SubDriveBase inst; return inst;}

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);
             
  void UpdateOdometry();
  void SyncSensors();
  frc::Rotation2d GetHeading();
  units::meters_per_second_t GetVelocity();

  static constexpr units::meters_per_second_t MAX_VELOCITY = 3_mps;
  static constexpr units::radians_per_second_t MAX_ANGULAR_VELOCITY =
      180_deg_per_s;

  void ResetGyroHeading();

 private:
  SubDriveBase();
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  AHRS m_gyro{frc::SerialPort::kMXP};

  frc::Translation2d m_frontLeftLocation{+0.281_m, -0.281_m};
  frc::Translation2d m_frontRightLocation{+0.281_m, +0.281_m};
  frc::Translation2d m_backLeftLocation{-0.281_m, -0.281_m};
  frc::Translation2d m_backRightLocation{-0.281_m, +0.281_m};

  const double FRONT_LEFT_MAG_OFFSET = 16.00;//-13.97;//-166.9;
  const double FRONT_RIGHT_MAG_OFFSET = -136.14;//-111.30;//108.5;
  const double BACK_LEFT_MAG_OFFSET = 108.63;//-149.68;//148.7;
  const double BACK_RIGHT_MAG_OFFSET = -31.64;//-44.82;//-136.05;

  SwerveModule m_frontLeft{canid::tfxDriveBaseFrontLeftDrive, canid::tfxDriveBaseFrontLeftTurn, canid::tfxDriveBaseFrontLeftEncoder, FRONT_LEFT_MAG_OFFSET};
  SwerveModule m_frontRight{canid::tfxDriveBaseFrontRightDrive, canid::tfxDriveBaseFrontRightTurn, canid::tfxDriveBaseFrontRightEncoder, FRONT_RIGHT_MAG_OFFSET};
  SwerveModule m_backLeft{canid::tfxDriveBaseBackLeftDrive, canid::tfxDriveBaseBackLeftTurn, canid::tfxDriveBaseBackLeftEncoder, BACK_LEFT_MAG_OFFSET};
  SwerveModule m_backRight{canid::tfxDriveBaseBackRightDrive, canid::tfxDriveBaseBackRightTurn, canid::tfxDriveBaseBackRightEncoder, BACK_RIGHT_MAG_OFFSET};

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, m_gyro.GetRotation2d()};

  frc::SwerveDrivePoseEstimator<4> _poseEstimator{
      frc::Rotation2d(), frc::Pose2d(), m_kinematics, 
      {0.0,0.0,0.0}, {0.00}, {0.0,0.0,0.0} 
  };
  frc::Pose2d _prevPose; // Used for velocity calculations

  frc::Field2d _fieldDisplay;
};

