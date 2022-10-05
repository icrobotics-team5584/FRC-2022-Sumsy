// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SubDriveBase.h"
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>

SubDriveBase::SubDriveBase(){
  frc::SmartDashboard::PutData("field", &_fieldDisplay);
}

// This method will be called once per scheduler run
void SubDriveBase::Periodic() {
  frc::SmartDashboard::PutNumber("heading", GetHeading().Degrees().value());
  frc::SmartDashboard::PutNumber("gyro", m_gyro.GetAngle());
  UpdateOdometry();
}

void SubDriveBase::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative) {
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, GetHeading())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_backLeft.SetDesiredState(bl);
  m_backRight.SetDesiredState(br);

  if (!frc::RobotBase::IsReal()) {
    double degPer20MS = units::degrees_per_second_t(rot).value() / 20;
    m_gyro.SetAngleAdjustment(GetHeading().Degrees().value() + degPer20MS);
  }
}

void SubDriveBase::SyncSensors() {
  m_frontLeft.SyncSensors();
  m_frontRight.SyncSensors();
  m_backLeft.SyncSensors();
  m_backRight.SyncSensors();
  m_gyro.Calibrate();
}

frc::Rotation2d SubDriveBase::GetHeading() {
  return units::degree_t{frc::InputModulus(m_gyro.GetAngle(), -180.0, 180.0)};
}

void SubDriveBase::UpdateOdometry() {
  auto fl = m_frontLeft.GetState();
  auto fr = m_frontLeft.GetState();
  auto bl = m_frontLeft.GetState();
  auto br = m_frontLeft.GetState();
  _poseEstimator.Update(GetHeading(), fl, fr, bl, br);
  _fieldDisplay.SetRobotPose(_poseEstimator.GetEstimatedPosition());
}

void SubDriveBase::ResetGyroHeading() {
  m_gyro.ZeroYaw();
}