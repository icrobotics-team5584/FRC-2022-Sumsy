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
  void SendSensorsToDash();
  void SetDesiredAngle(frc::Rotation2d angle);
  frc::Rotation2d GetAngle();


 private:
  const int TICS_PER_MOTOR_REVOLUTION = 2048;
  const int CANCODER_TICS_PER_MOTOR_REVOLUTION = 2048;
  const double TURNING_GEAR_RATIO = 150.0/7.0;
  const double TICS_PER_TURNING_WHEEL_REVOLUTION =
      TICS_PER_MOTOR_REVOLUTION * TURNING_GEAR_RATIO;

  static constexpr double kWheelRadius = 0.0508;
  static constexpr double kRotationConversion =
      10 / 2048 / 6.71 * wpi::numbers::pi * 0.1016;
  static constexpr auto kModuleMaxAngularVelocity =
      wpi::numbers::pi * 1_rad_per_s;  // radians per second
  static constexpr auto kModuleMaxAngularAcceleration =
      wpi::numbers::pi * 2_rad_per_s / 1_s;  // radians per second^2

  const double TURN_P = 0.2;
  const double TURN_I = 0.0;
  const double TURN_D = 0.1;
  const int PID_SLOT_INDEX = 0;

  TalonFX _canDriveMotor;
  TalonFX _canTurnMotor;

  ctre::phoenix::sensors::CANCoder _canTurnEncoder;
};
