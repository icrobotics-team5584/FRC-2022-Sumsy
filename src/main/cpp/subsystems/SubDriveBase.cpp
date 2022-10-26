// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SubDriveBase.h"
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>

SubDriveBase::SubDriveBase(){
  m_gyro.Calibrate();
  frc::SmartDashboard::PutData("field", &_fieldDisplay);
}

// This method will be called once per scheduler run
void SubDriveBase::Periodic() {
  frc::SmartDashboard::PutNumber("heading", GetHeading().Degrees().value());
  frc::SmartDashboard::PutNumber("gyro", m_gyro.GetAngle());
  frc::SmartDashboard::PutBoolean("gyro is callibrating", m_gyro.IsCalibrating());
  frc::SmartDashboard::PutNumber("Drivebase speed", GetVelocity().value());

  UpdateOdometry();
}

void SubDriveBase::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot, bool fieldRelative) {

  // Get states of all swerve modules
  auto states = m_kinematics.ToSwerveModuleStates( fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, GetHeading())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  // Set speed limit and apply speed limit to all modules
  m_kinematics.DesaturateWheelSpeeds(&states, MAX_VELOCITY);

  // Setting modules from aquired states
  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);

  // Check if robot is in simulation. 
  // Manualy adjusting gyro by calculating rotation in simulator as gyro is not enabled in simulation
  if (frc::RobotBase::IsSimulation()) {
    double degPer20MS = units::degrees_per_second_t(rot).value() / 20;
    m_gyro.SetAngleAdjustment(GetHeading().Degrees().value() + degPer20MS);
  }
}

// Syncs encoder values when the robot is turned on
void SubDriveBase::SyncSensors() {
  m_frontLeft.SyncSensors();
  m_frontRight.SyncSensors();
  m_backLeft.SyncSensors();
  m_backRight.SyncSensors();
  m_gyro.Calibrate();
}

// Convertion from 0-360 from gyro to -180 to 180
frc::Rotation2d SubDriveBase::GetHeading() {
  return units::degree_t{frc::InputModulus(m_gyro.GetAngle(), -180.0, 180.0)};
}

// Calculate robot's velocity over past time step (20 ms)
units::meters_per_second_t SubDriveBase::GetVelocity() {
  auto robotDisplacement = _prevPose
    .Translation()
    .Distance(_poseEstimator
      .GetEstimatedPosition()
      .Translation()
    );
  return units::meters_per_second_t{robotDisplacement/20_ms};
}

// calculates the relative field location
void SubDriveBase::UpdateOdometry() {
  auto fl = m_frontLeft.GetState();
  auto fr = m_frontRight.GetState();
  auto bl = m_backLeft.GetState();
  auto br = m_backRight.GetState();
  _prevPose = _poseEstimator.GetEstimatedPosition();
  _poseEstimator.Update(GetHeading(), fl, fr, bl, br);
  _fieldDisplay.SetRobotPose(_poseEstimator.GetEstimatedPosition());
}

void SubDriveBase::ResetGyroHeading() {
  m_gyro.Reset();
}
