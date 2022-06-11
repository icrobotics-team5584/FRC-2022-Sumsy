// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <frc/geometry/Rotation2d.h>

#include <memory>
#include <wpi/numbers>

#include "utilities/FalconFactory.h"

class SwerveModule {
 public:
  SwerveModule(int canDriveMotorID, int canTurnMotorID, int canTurnEncoderID);

  void SetDesiredState(const frc::SwerveModuleState& state);
  void ZeroSensors();
  void SyncSensors();
  void SendSensorsToDash();
  void SetDesiredAngle(frc::Rotation2d angle);
  void SetDesiredVelocity(units::meters_per_second_t velocity);
  frc::Rotation2d GetAngle();

 private:
  const int TICS_PER_MOTOR_REVOLUTION = 2048;
  const double TURNING_GEAR_RATIO = 150.0/7.0;
  const double TICS_PER_TURNING_WHEEL_REVOLUTION =
      TICS_PER_MOTOR_REVOLUTION * TURNING_GEAR_RATIO;
  const units::meter_t WHEEL_RADIUS = 0.0508_m;

  const double TURN_P = 0.2;
  const double TURN_I = 0.0;
  const double TURN_D = 0.1;
  const double DRIVE_P = 0.1;
  const double DRIVE_I = 0.0;
  const double DRIVE_D = 0.0;
  const int PID_SLOT_INDEX = 0;

  TalonFX _canDriveMotor;
  TalonFX _canTurnMotor;
  CANCoder _canTurnEncoder;
};
