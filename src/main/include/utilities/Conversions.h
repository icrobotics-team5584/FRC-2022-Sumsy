#pragma once

#include <units/velocity.h>
#include <units/length.h>
#include <units/angle.h>
#include <frc/geometry/Rotation2d.h>

class Conversions {
 public:
  static const int FALCON_TICS_PER_REVOLUTION = 2048;
  
  static units::meters_per_second_t FalconVelToMPS(int falconVelocity, units::meter_t wheelRadius) {
    const double ticsPerMS = falconVelocity/100;
    const double ticsPerSecond = ticsPerMS*1000;
    const double revsPerSecond = ticsPerSecond/FALCON_TICS_PER_REVOLUTION;
    const double metersPerSecond = revsPerSecond * wheelRadius.value();
    return units::meters_per_second_t{metersPerSecond};
  }

  static units::degree_t TicsToDegrees(const int tics, const double ticsPerRevolution) {
    const double positionInRevolutions = tics / ticsPerRevolution;
    const double positionInDegrees = positionInRevolutions * 360; 
    return units::degree_t{positionInDegrees};
  }

  static frc::Rotation2d TicsToRotation2d(const int tics, const double ticsPerRevolution) {
    return frc::Rotation2d(TicsToDegrees(tics, ticsPerRevolution));
  }

 private:
  Conversions();
};
