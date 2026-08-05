// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.common.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

public class MAXSwerveModule {
  private final SparkFlex m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /**
   * Returns the most recently commanded state of the module, in chassis-relative terms.
   *
   * <p>Exposed so the drivetrain can log desired-vs-actual per module, which is what makes
   * drive PID and feedforward tuning possible from a log file.
   *
   * @return The desired state of the module.
   */
  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  /**
   * Commands the drive motor open loop while holding the module at a fixed angle.
   *
   * <p>For feedforward characterisation only. Closed-loop velocity control cannot be used to
   * measure kS and kV, because the loop compensates for exactly the relationship being
   * measured — the sweep has to see the raw voltage-to-velocity response.
   *
   * @param dutyCycle Motor output, −1..1.
   * @param angle     Chassis-relative angle to hold the module at.
   */
  public void setDriveOpenLoop(double dutyCycle, Rotation2d angle) {
    Rotation2d corrected = angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
    m_turningClosedLoopController.setSetpoint(corrected.getRadians(), ControlType.kPosition);
    m_drivingSpark.set(dutyCycle);
  }

  /** @return the drive motor's applied output voltage. */
  public double getDriveVoltage() {
    return m_drivingSpark.getAppliedOutput() * m_drivingSpark.getBusVoltage();
  }

  /** @return the drive wheel's measured velocity in metres per second. */
  public double getDriveVelocity() {
    return m_drivingEncoder.getVelocity();
  }

  /**
   * Re-applies the drive closed-loop gains without touching any other configuration.
   *
   * <p>For live tuning only. Uses no-reset, no-persist so it is a cheap in-memory tweak
   * rather than a full reconfiguration and a flash write — the latter would wear the
   * controller and stall the CAN bus if done repeatedly.
   *
   * @param p proportional gain.
   * @param d derivative gain.
   */
  public void applyDriveGains(double p, double d) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop.p(p).d(d);
    m_drivingSpark.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Re-applies the steering closed-loop gains without touching any other configuration.
   *
   * @param p proportional gain.
   * @param d derivative gain.
   */
  public void applyTurnGains(double p, double d) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.p(p).d(d);
    m_turningSpark.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Drives the module at a fixed <b>voltage</b>, wheel held at the given angle.
   *
   * <p>For SysId, which commands volts rather than duty cycle. The distinction matters: duty cycle
   * is a fraction of whatever the bus happens to be, so a duty-cycle sweep silently rescales itself
   * as the battery sags and the resulting fit is against a moving input. {@code setVoltage}
   * compensates for bus voltage, so 6 V means 6 V at the start of the run and at the end of it.
   *
   * @param volts Output voltage.
   * @param angle Chassis-relative angle to hold the module at.
   */
  public void setDriveVoltage(double volts, Rotation2d angle) {
    Rotation2d corrected = angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
    m_turningClosedLoopController.setSetpoint(corrected.getRadians(), ControlType.kPosition);
    m_drivingSpark.setVoltage(volts);
  }

  /** @return the drive wheel's measured position in metres. */
  public double getDrivePositionMeters() {
    return m_drivingEncoder.getPosition();
  }

  /** @return the drive motor's output current in amps. */
  public double getDriveCurrent() {
    return m_drivingSpark.getOutputCurrent();
  }

  /**
   * Re-applies the drive smart current limit without touching any other configuration.
   *
   * <p>For the traction calibration, which steps the limit upward until the wheels break loose.
   * No-persist, like the gain setters: the limit is stepped many times in one run and each persisted
   * write is a flash erase cycle on the controller.
   *
   * <p>Because it does not persist, the limit reverts to whatever
   * {@code SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT} says on the next power cycle. That is the
   * intended behaviour — a calibration run must not be able to leave the robot configured for a limit
   * nobody chose.
   *
   * @param amps Smart current limit in amps.
   */
  public void applyDriveCurrentLimit(int amps) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.smartCurrentLimit(amps);
    m_drivingSpark.configure(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Puts both of this module's motors in coast, so they can be turned by hand, or back in brake.
   *
   * <p>For the hand-motion polarity check, which is the one calibration step that runs with the
   * motors unpowered. Brake mode shorts the windings, so a braked module can be forced round but
   * fights the whole way and reports a jerky, stiction-dominated position trace — which is exactly
   * the signal the polarity check is trying to read.
   *
   * <p>No-persist, deliberately. A run that is interrupted at the wrong moment therefore cannot leave
   * the drivetrain coasting for the next match: a power cycle restores brake even if nothing else
   * does. The routine also restores it explicitly, but that is the second line of defence rather than
   * the only one.
   *
   *
   * <p><b>Entering coast also stops the motors, and it has to.</b> Idle mode only decides what happens
   * when a controller is applying nothing. A SPARK holding a closed-loop reference is not idle, and it
   * carries on servoing to that reference whether it is set to coast or brake — so setting coast alone
   * would produce a mechanism that still fights the hand moving it, and a polarity reading taken from a
   * motor that was driving itself. Stopping the output is what actually makes it free.
   *
   * @param coast True for coast, false to restore brake.
   */
  public void setCoastForHandCalibration(boolean coast) {
    if (coast) {
      // Before the idle mode, not after: between the two writes the module is still executing its
      // last steering reference, and doing it in this order keeps that window as short as possible.
      m_drivingSpark.stopMotor();
      m_turningSpark.stopMotor();
    }

    IdleMode mode = coast ? IdleMode.kCoast : IdleMode.kBrake;

    SparkFlexConfig driveConfig = new SparkFlexConfig();
    driveConfig.idleMode(mode);
    m_drivingSpark.configure(
        driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    SparkMaxConfig turnConfig = new SparkMaxConfig();
    turnConfig.idleMode(mode);
    m_turningSpark.configure(
        turnConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** @return the turning encoder reading, in radians, without the chassis angular offset. */
  public double getRawTurnPositionRadians() {
    return m_turningEncoder.getPosition();
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
