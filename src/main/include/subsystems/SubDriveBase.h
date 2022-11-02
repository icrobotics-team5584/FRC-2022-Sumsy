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
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/controller/HolonomicDriveController.h>

#include "Constants.h"
#include "utilities/SwerveModule.h"

class SubDriveBase : public frc2::SubsystemBase {
 public:
  SubDriveBase();

   static SubDriveBase &GetInstance()
  {
    static SubDriveBase inst;
    return inst;
  }
  void UpdatePidControllerDrive();
  void DriveToTarget(units::meter_t xDistance, units::meter_t yDistance, units::meter_t targetDistance, units::degree_t targetRotation);

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);

  void DriveToPathPoint(frc::Pose2d& pos, units::meters_per_second_t vel, frc::Rotation2d& rot);

  frc::Pose2d GetPose();
  void DisplayPose(std::string label, frc::Pose2d pose);
             
  void UpdateOdometry();
  void SyncSensors();
  frc::Rotation2d GetHeading();

  static constexpr units::meters_per_second_t kMaxSpeed =
      3_mps;  // 3 meters per second
  static constexpr units::radians_per_second_t kMaxAngularSpeed{
      wpi::numbers::pi};

  static constexpr units::radians_per_second_squared_t kMaxAngularAcc{
      wpi::numbers::pi};

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void ResetGyroHeading();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
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

   

  AHRS m_gyro{frc::SerialPort::kMXP};

  frc::SwerveDriveKinematics<4> m_kinematics{
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,
      m_backRightLocation};

  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, m_gyro.GetRotation2d()};

  frc::PIDController Xcontroller{0.1,0,0};
  frc::PIDController Ycontroller{0.1,0,0};
  frc::ProfiledPIDController<units::radian> Rcontroller{1.8,0,0,{kMaxAngularSpeed, kMaxAngularAcc}};
  frc::HolonomicDriveController _driveController{Xcontroller, Ycontroller, Rcontroller};

  frc::SwerveDrivePoseEstimator<4> _poseEstimator{
      frc::Rotation2d(), frc::Pose2d(), m_kinematics, 
      {0.0,0.0,0.0}, {0.00}, {0.0,0.0,0.0} 
  };

  frc::Field2d _fieldDisplay;
};

