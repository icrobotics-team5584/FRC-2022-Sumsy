// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SubDriveBase.h"

SubDriveBase::SubDriveBase() {
  frc::SmartDashboard::PutNumber("set desired angle", 0);
};

// This method will be called once per scheduler run
void SubDriveBase::Periodic() {
  m_frontLeft.SendSensorsToDash();
  // m_frontRight.SendSensorsToDash();
  // m_backLeft.SendSensorsToDash();
  // m_backRight.SendSensorsToDash();

  frc::Rotation2d desiredAngle = units::degree_t(frc::SmartDashboard::GetNumber("set desired angle", 0));
  m_frontLeft.SetDesiredAngle(desiredAngle);
}

void SubDriveBase::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot, bool fieldRelative) {
  auto states = m_kinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  //m_frontLeft.SetDesiredState(fl);
  //m_frontRight.SetDesiredState(fr);
  //m_backLeft.SetDesiredState(bl);
  //m_backRight.SetDesiredState(br);
}


void SubDriveBase::ZeroSensors() {
  m_frontLeft.ZeroSensors();
  m_frontRight.ZeroSensors();
  m_backLeft.ZeroSensors();
  m_backRight.ZeroSensors();
}