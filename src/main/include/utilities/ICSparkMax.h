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
#include <units/current.h>
#include <units/velocity.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>

/**
 * Wrapper around the SPARK MAX class that adds better simulation support and
 * some convenience features. Unlike the rev::CANSparkMax class, all encoder and
 * PID functionality is built in, you do not need to create sperate objects.
 * A sensible current limit is also automatically set depending on which motor
 * Type is used.
 */
class ICSparkMax : public rev::CANSparkMax, wpi::Sendable {
 public:
  /**
   * Create a new object to control a SPARK MAX motor controller, with
   * added convenience features.
   *
   * @param deviceID The device CAN id
   * @param currentLimit Value used for spark max smart current limiting
   */
  ICSparkMax(int deviceID, units::ampere_t currentLimit = 20_A);

  /**
   * Sets the closed loop target (aka reference or goal) for the motor to
   * drive to.
   *
   * @param target The target to set depending on the control mode. For
   * basic duty cycle control this should be a value between -1 and 1
   * Otherwise: Voltage Control: Voltage (volts) Velocity Control: Velocity
   * (RPM) Position Control: Position (Rotations) Current Control: Current
   * (Amps). The units can be changed for position and velocity by a scale
   * factor using SetConversionFactors().
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
                 double arbFeedForward = 0.0);

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
   * Get the velocity of the motor. This returns the native units
   * of 'RPM' by default, and can be changed by a scale factor
   * using SetConversionFactors().
   *
   * @return Number the RPM of the motor
   */
  double GetVelocity();

  /**
   * Get the position of the motor. This returns the native units
   * of 'rotations' by default, and can be changed by a scale factor
   * using SetConversionFactors().
   *
   * @return Number of rotations of the motor
   */
  double GetPosition() { return _encoder->GetPosition(); };

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

  /**
   * Configure the maximum acceleration of the SmartMotion mode. This is the
   * accleration that the motor velocity will increase at until the max
   * velocity is reached
   *
   * @param maxAcceleration The maxmimum acceleration for the motion profile.
   * In RPM per second by default and can be scaled with SetConversionFactors().
   */
  void SetSmartMotionMaxAccel(double maxAcceleration);

  /**
   * Configure the maximum velocity of the SmartMotion mode. This is the
   * velocity that is reached in the middle of the profile and is what the
   * motor should spend most of its time at
   *
   * @param maxVelocity The maxmimum cruise velocity for the motion profile.
   * In RPM by default and can be scaled with SetConversionFactors().
   */
  void SetSmartMotionMaxVelocity(double maxVelocity);

  /**
   * Set the conversion factor for position, velocity and acceleration of the
   * encoder. The native units of rotations, RPM and RPM per second are
   * multiplied by these values before being used or returned.
   */
  void SetConversionFactors(double rotationsToDesired, double RPMToDesired,
                            double RPMpsToDesired);

  /**
   * Set the Proportional, Integral and Derivative Gain constants of the PIDF
   * controller on the SPARK MAX. This uses the Set Parameter API and should be
   * used infrequently. The parameters do not presist unless burnFlash() is
   * called.
   *
   * @param P The proportional gain value, must be positive
   * @param I The Integral gain value, must be positive
   * @param D The Derivative gain value, must be positive
   * @param FF The Feed Forward gain value, must be positive. This is multiplied
   * by the target before being added to the final output power.
   */
  void SetPIDFF(double P, double I, double D, double FF = 0.0);

  /**
   * Set the position of the encoder.
   *
   * @param position Number of rotations of the motor
   */
  void SetEncoderPosition(double position);

  /**
   * Set the min amd max output for the closed loop mode.
   *
   * This uses the Set Parameter API and should be used infrequently.
   * The parameter does not presist unless burnFlash() is called.
   *
   * @param minOutputPercent Reverse power minimum to allowed
   *
   * @param maxOutputPercent Forward power maximum to allowed
   */
  void SetClosedLoopOutputRange(double minOutputPercent,
                                double maxOutputPercent);

  /**
   * Configure the allowed closed loop error of SmartMotion mode. This value
   * is how much deviation from your setpoint is tolerated and is useful in
   * preventing oscillation around your setpoint. When the true position is
   * within tolerance, no power will be applied to the motor.
   */
  void SetSmartMotionTolerance(double tolerance) {
    _pidController.SetSmartMotionAllowedClosedLoopError(tolerance);
  };

  /**
   * Switch to using an external encoder connected to the alternate encoder data
   * port on the SPARK MAX. The pins on this port are defined as:
   *
   * Pin 4 (Forward Limit Switch): Index
   * Pin 6 (Multi-function): Encoder A
   * Pin 8 (Reverse Limit Switch): Encoder B
   *
   * This call will disable support for the limit switch inputs.
   *
   * @param countsPerRev The number of encoder counts per revolution. Leave as
   * default for the REV through bore encoder.
   */
  void UseAlternateEncoder(int countsPerRev = 8192);

  // Sendable setup, called when this is passed into smartDashbaord::PutData()
  void InitSendable(wpi::SendableBuilder& builder) override;

  // Delete some SPARK MAX functions so user doesn't get multiple copies of
  // friend objects.
  rev::SparkMaxRelativeEncoder GetEncoder() = delete;
  rev::SparkMaxAlternateEncoder GetAlternateEncoder() = delete;
  rev::SparkMaxPIDController GetPIDController() = delete;

 private:
  using Mode = rev::CANSparkMax::ControlType;

  // Related REVLib objects
  rev::SparkMaxPIDController _pidController{CANSparkMax::GetPIDController()};
  std::unique_ptr<rev::RelativeEncoder> _encoder =
      std::make_unique<rev::SparkMaxRelativeEncoder>(CANSparkMax::GetEncoder());
  double _RPMpsToDesiredAccelUnits = 1.0;

  // PID simulation configuration
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
