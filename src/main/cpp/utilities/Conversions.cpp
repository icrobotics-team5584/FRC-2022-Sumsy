#include "utilities/Conversions.h"

#include <units/angular_velocity.h>
#include <units/angle.h>

namespace Conversions {

units::meters_per_second_t FalconVelToRobotVel(int falconVelocity,
                                               double gearRatio,
                                               units::meter_t wheelRadius) {
  const units::meter_t wheelCircumference = wheelRadius * TAU;
  const double motorRevsPer100ms = falconVelocity / FALCON_TICS_PER_REVOLUTION;
  const double shaftRevsPer100ms = motorRevsPer100ms / gearRatio;
  const double shaftRevsPerMS = shaftRevsPer100ms / 100;
  const double shaftRevsPerSecond = shaftRevsPerMS * 1000;
  const units::meters_per_second_t robotVelocity{shaftRevsPerSecond *
                                                 wheelCircumference.value()};
  return robotVelocity;
}

double RobotVelToFalconVel(units::meters_per_second_t robotVelocity,
                           double gearRatio, units::meter_t wheelRadius) {
  const units::meter_t wheelCircumference = wheelRadius * TAU;
  const double metersPerMS = robotVelocity.value() / 1000;
  const double metersPer100MS = metersPerMS * 100;
  const double shaftTurnsPer100MS = metersPer100MS / wheelCircumference.value();
  const double motorTurnsPer100MS = shaftTurnsPer100MS * gearRatio;
  const double ticsPer100MS = motorTurnsPer100MS * FALCON_TICS_PER_REVOLUTION;
  return ticsPer100MS;
}

frc::Rotation2d FalconTicsToOutputRotations(const int tics,
                                            const double gearRatio) {
  const double ticsPerOutputRevolution = FALCON_TICS_PER_REVOLUTION * gearRatio;
  const units::turn_t outputRevolutions{tics / ticsPerOutputRevolution};
  return units::degree_t{outputRevolutions};
}

}  // namespace Conversions