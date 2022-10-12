// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SubDriveBase.h"
#include <frc/MathUtil.h>

SubDriveBase::SubDriveBase(){
}

// This method will be called once per scheduler run
void SubDriveBase::Periodic() {
  m_frontLeft.SendSensorsToDash();
  m_frontRight.SendSensorsToDash();
  m_backLeft.SendSensorsToDash();
  m_backRight.SendSensorsToDash();

  UpdatePidControllerDrive();
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

void SubDriveBase::DriveToTarget(units::meter_t xDistance, units::meter_t yDistance, units::meter_t targetDistance) {
   double speed = Xcontroller.Calculate(xDistance.value(), targetDistance.value());
   Drive(speed*1_mps, 0_mps, 0_rad_per_s, false);
}

//void SubDriveBase::SetTargetRpm(double rpm){
   // _controller.SetSetpoint(rpm);
//}

void SubDriveBase::UpdatePidControllerDrive() {
    /*double feedForward = (1.0f/5300.0f)* _controller.GetSetpoint();
    double _output = _controller.Calculate(_encShooter1.GetVelocity()) + feedForward;
    if (_output >= 0) {
        _spmShooter1.SetVoltage(units::volt_t(_output*12)); 
    } else {
       // _spmShooter1.Set(0);
    } */
}
//void SubShooter::Stop() {
   // SetTargetRpm(0);
   //_shouldTrackTarget = false;
//}