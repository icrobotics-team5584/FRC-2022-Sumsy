#pragma once

#include <frc/Notifier.h>
#include <frc/Timer.h>
#include <frc/controller/PIDController.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <hal/simulation/SimDeviceData.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>

class ICSparkMax : public rev::CANSparkMax, wpi::Sendable {
 public:
  // Type of motor in use, only used for default current limiting setup
  enum class Type { NEO, NEO_550 };

  /**
   * Create a new object to control a SPARK MAX motor controller, with
   * added convenience features.
   *
   * @param deviceID The device CAN id
   * @param type Either NEO or NEO_550. Determines default current limiting.
   */
  ICSparkMax(int deviceID, Type type);

  /**
   * Sets the closed loop target (aka reference or goal) for the motor to
   * drive to.
   *
   * @param target The target to set depending on the control mode. For
   * basic duty cycle control this should be a value between -1 and 1
   * Otherwise: Voltage Control: Voltage (volts) Velocity Control: Velocity
   * (RPM) Position Control: Position (Rotations) Current Control: Current
   * (Amps). The units can be changed for position and velocity by a scale
   * factor using setPositionConversionFactor() and
   * setVelocityConversionFactor().
   *
   * @param controlType Is the control type
   *
   * @param pidSlot 0, 1, or 2 to decide which PID config to use for this
   * command
   *
   * @param arbFeedforward A value from -32.0 to 32.0 which is a voltage
   * applied to the motor after the result of the specified control mode. The
   * units for the parameter is Volts. This value is set after the control
   * mode, but before any current limits or ramp rates
   */
  void SetTarget(double target, rev::CANSparkMax::ControlType controlType,
                 int pidSlot = 0, double arbFeedForward = 0.0);

  /**
   * Gets the current closed loop target. This will be in RPM for velocity
   * control, rotations for position control, or 0.0 to 1.0 for duty cycle
   * (power percentage) control. These units can be changed by a scale
   * factor using setPositionConversionFactor() and
   * setVelocityConversionFactor().
   */
  double GetTarget() { return _target; };

  /**
   * Calculates how much voltage the spark max would be giving to the
   * attached motor given its current control type and PID configuration.
   * Use this in conjunction with one of WPILib's physics simulation classes.
   * (https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html)
   */
  units::volt_t GetSimVoltage();

  /**
   * Due to REVLib archetecture, we need to calculate our own power output
   * when in simulation using a stored copy of the PID configuration.
   * Call this after changing PID configuration to keep the simulation in
   * sync with the Spark Max's internal values.
   * This is called automatically when SetTarget() is called.
   */
  void SyncSimPID();

  /**
   * It is the user's responsibility to update the encoder position and
   * velocity when in simulation. To do this, use WPILib's physics simulation
   * classes at
   * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html
   * to get the position and velocity of the mechanism attached to this motor.
   */
  void UpdateSimEncoder(double position, double velocity);

  /**
   * Gets the current closed loop control type.
   */
  rev::CANSparkMax::ControlType GetControlType() { return _controlType; };

  /**
   * Common interface to stop the motor until Set is called again or
   * closed loop control is started.
   */
  void StopMotor() override;

  /**
   * Common interface for setting the speed of a speed controller.
   *
   * @param speed The speed to set.  Value should be between -1.0 and 1.0.
   */
  void Set(double speed) override;

  /**
   * Common interface for setting the voltage of a speed controller.
   *
   * @param output The voltage to set.
   */
  void SetVoltage(units::volt_t output) override;

  // Sendable setup, called when this is passed into smartDashbaord::PutData()
  void InitSendable(wpi::SendableBuilder& builder) override;

  void SetSmartMotionMaxAccel(double maxAcceleration);

  void SetSmartMotionMaxVelocity(double maxVelocity);

  void SetConversionFactors(double rotationsToDesired, double RPMToDesired,
                            double RPMpsToDesired);

  void SetPIDFF(double P, double I, double D, double FF = 0.0);

  void SetEncoderPosition(double position);

  void SetClosedLoopOutputRange(double minOutputPercent,
                                double maxOutputPercent);

  void SetSmartMotionTolerance(double tolerance) {
    _pidController.SetSmartMotionAllowedClosedLoopError(tolerance);
  };

  rev::SparkMaxRelativeEncoder GetEncoder() = delete;
  rev::SparkMaxPIDController GetPIDController() = delete;

 private:
  using Mode = rev::CANSparkMax::ControlType;

  // Default smart Current limiting config
  Type _type;
  unsigned const int NEO_CURRENT_LIMIT = 40;
  unsigned const int NEO_550_CURRENT_LIMIT = 20;

  // Related REVLib objects
  rev::SparkMaxPIDController _pidController{CANSparkMax::GetPIDController()};
  rev::SparkMaxRelativeEncoder _encoder{CANSparkMax::GetEncoder()};
  double _prevEncoderPos = 0.0;
  double _RPMpsToDesiredAccelUnits = 1.0;

  // PID simulation configuration
  int _pidSlot = 0;
  bool _updatingTargetFromSendable = false;
  double _target = 0;
  double _arbFeedForward = 0.0;
  frc::PIDController _simController{0, 0, 0};
  frc::TrapezoidProfile<units::meters> _simSmartMotionProfile{
      {0_mps, 0_mps_sq},  // constraints
      {0_m, 0_mps}        // goal states
  };
  frc::Timer _smartMotionProfileTimer;
  Mode _controlType = Mode::kDutyCycle;
  void SetInternalControlType(Mode controlType);
  void GenerateSMProfile();
  double GetCurrentSMVelocity();

  // Sim device values (stuff that shows under Other Devices on Glass)
  frc::sim::SimDeviceSim _simDeviceSim{"SPARK MAX ", GetDeviceId()};
  hal::SimDouble _simVelocity = _simDeviceSim.GetDouble("Velocity");
  hal::SimDouble _simPosition = _simDeviceSim.GetDouble("Position");
  hal::SimInt _simControlMode = _simDeviceSim.GetInt("Control Mode");
};
