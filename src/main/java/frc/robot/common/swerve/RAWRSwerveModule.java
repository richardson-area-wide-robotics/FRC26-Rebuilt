package frc.robot.common.swerve;

// Copyright (c) LASA Robotics and other contributors
// Open Source Software; you can modify and/or share it under the terms of
// the MIT license file in the root directory of this project.

import java.time.Duration;
import java.time.Instant;
import java.util.Map;
import java.lang.Runtime;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import frc.robot.CommonConstants;
import frc.robot.common.components.hardware.SwerveModuleHardware;
import org.lasarobotics.drive.TractionControlController;
import org.lasarobotics.drive.swerve.DriveWheel;
import org.lasarobotics.drive.swerve.SwerveModule;
import org.lasarobotics.drive.swerve.SwerveModuleSim;
import org.lasarobotics.drive.swerve.parent.REVSwerveModule;
import org.lasarobotics.hardware.PurpleManager;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;
import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.GlobalConstants;
import org.lasarobotics.utils.PIDConstants;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import  com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.common.components.RobotUtils;

/** REV MAXSwerve module */
public class RAWRSwerveModule extends SwerveModule implements Sendable {

  private final double DRIVE_TICKS_PER_METER;
  private final double DRIVE_METERS_PER_TICK;
  private final double DRIVE_METERS_PER_ROTATION;
  private final double DRIVE_MAX_LINEAR_SPEED;

  private final Spark driveMotor;
  private final Spark rotateMotor;
  private final SwerveModuleSim moduleSim;
  private SimpleMotorFeedforward driveFF;
  private final Rotation2d zeroOffset;
  
  private final SwerveModule.Location location;
  private Rotation2d previousRotatePosition;

  private double simDrivePosition;
  private volatile SwerveModulePosition simModulePosition;
  private volatile SwerveModuleState desiredState;

  private final double autoLockTime;

  private Instant autoLockTimer;

  private final ExecutorService EXECUTOR_SERVICE = Executors.newFixedThreadPool(2);  
    
  public static final Map<SwerveModule.Location, Angle> ZERO_OFFSET = Map.ofEntries(
    Map.entry(SwerveModule.Location.LeftFront, Units.Radians.of(Math.PI / 2)),
    Map.entry(SwerveModule.Location.RightFront, Units.Radians.zero()),
    Map.entry(SwerveModule.Location.LeftRear, Units.Radians.of(Math.PI)),
    Map.entry(SwerveModule.Location.RightRear, Units.Radians.of(Math.PI / 2))
  );
  
  
    /** Make a {@link REVSwerveModule}
     * 
     * @param driveMotor the drive motor (Ex: forward-backword)
     * @param rotateMotor the rotate motor (Ex: left-right)
     * @param location the location of the swerve module (Ex: front left)
     * 
     * @author Hudson Strub
     * @since 2025
     */
    public static RAWRSwerveModule createSwerve(Spark.ID driveMotor, Spark.ID rotateMotor, SwerveModule.Location location){
      
      SwerveModuleHardware swerveModuleHardware = new SwerveModuleHardware(
                    new Spark(driveMotor, Spark.MotorKind.NEO_VORTEX),
                    new Spark(rotateMotor, Spark.MotorKind.NEO_550)
            );
    
      
      RAWRSwerveModule swerveModule = new RAWRSwerveModule(
            swerveModuleHardware,
            location,
            SwerveModule.MountOrientation.STANDARD,
            SwerveModule.MountOrientation.INVERTED,
            CommonConstants.SwerveConstants.GEAR_RATIO,

            DriveWheel.create(
              Distance.ofRelativeUnits(75, Units.Millimeter), 
              Dimensionless.ofBaseUnits(1.6, Units.Value),
              Dimensionless.ofBaseUnits(1.3, Units.Value)), 
            ZERO_OFFSET.get(location),
            PIDConstants.of(0.18, 0, 0.174, 0, 0), // Replace with actual PID constants
            FFConstants.of(0, 0, 0, 0),
          PIDConstants.of(4.55, 0, 0.2, 0, 0), // The PID for the rotate Motor
          FFConstants.of(0, 0, 0, 0),  // Replace with actual feed-forward constants
          Dimensionless.ofBaseUnits(CommonConstants.DriveConstants.DRIVE_SLIP_RATIO, Units.Value),
          Mass.ofRelativeUnits(RobotUtils.getRobotConfig().massKG, Units.Kilograms),
          Distance.ofRelativeUnits(23, Units.Inches),
          Distance.ofRelativeUnits(24.5, Units.Inches),
          Time.ofBaseUnits(CommonConstants.DriveConstants.AUTO_LOCK_TIME, Units.Second));
    
    
    return swerveModule;
  }

  /**
   * Create an instance of a REV swerve module
   * @param swerveHardware Hardware devices required by swerve module
   * @param location Module location
   * @param motorOrientation Motor mount orientation
   * @param encoderOrientation Encoder orientation
   * @param gearRatio Module gear ratio
   * @param driveWheel Wheel installed in swerve module
   * @param zeroOffset Chassis zero offset due to module calibration position
   * @param drivePID Drive motor PID gains
   * @param driveFF Drive motor feed forward gains
   * @param rotatePID Rotate motor PID gains
   * @param rotateFF Rotate motor feed forward gains
   * @param slipRatio Desired slip ratio [1%, 40%]
   * @param mass Robot mass
   * @param wheelbase Robot wheelbase
   * @param trackWidth Robot track width
   * @param autoLockTime Time before automatically rotating module to locked position (10 seconds max)
   */
  private RAWRSwerveModule(SwerveModuleHardware swerveHardware,
                           SwerveModule.Location location,
                           SwerveModule.MountOrientation motorOrientation,
                           SwerveModule.MountOrientation encoderOrientation,
                           SwerveModule.GearRatio gearRatio,
                           DriveWheel driveWheel, Angle zeroOffset,
                           PIDConstants drivePID, FFConstants driveFF,
                           PIDConstants rotatePID, FFConstants rotateFF,
                           Dimensionless slipRatio, Mass mass,
                           Distance wheelbase, Distance trackWidth,
                           Time autoLockTime) {
    super(location, gearRatio, driveWheel, zeroOffset, wheelbase, trackWidth, swerveHardware.driveMotor().getID().name);

    //Get the encoder ticks per rotation for the drive motor
    int encoderTicksPerRotation = RobotUtils.getEncoderTicksPerRotation(swerveHardware.driveMotor());

    DRIVE_TICKS_PER_METER =
      (encoderTicksPerRotation * gearRatio.getDriveRatio())
      * (1 / (driveWheel.diameter.in(Units.Meters) * Math.PI));
    DRIVE_METERS_PER_TICK = 1 / DRIVE_TICKS_PER_METER;
    DRIVE_METERS_PER_ROTATION = DRIVE_METERS_PER_TICK * encoderTicksPerRotation;
    DRIVE_MAX_LINEAR_SPEED = (swerveHardware.driveMotor().getKind().getMaxRPM() / 60) * DRIVE_METERS_PER_ROTATION * CommonConstants.DriveConstants.DRIVETRAIN_EFFICIENCY;

    // Set traction control controller
    super.setTractionControlController(new TractionControlController(driveWheel, slipRatio, mass, Units.MetersPerSecond.of(DRIVE_MAX_LINEAR_SPEED)));

    this.driveMotor = swerveHardware.driveMotor();
    this.rotateMotor = swerveHardware.rotateMotor();
    this.moduleSim = new SwerveModuleSim(
      driveMotor.getKind().motor,
      driveFF.withKA((driveFF.kA <= 0.0) ? SwerveModule.MIN_SIM_kA : driveFF.kA),
      rotateMotor.getKind().motor, rotateFF.withKA((rotateFF.kA <= 0.0) ? SwerveModule.MIN_SIM_kA : rotateFF.kA)
    );
    this.driveFF = new SimpleMotorFeedforward(driveFF.kS, driveFF.kV, driveFF.kA);
    this.location = location;
    this.zeroOffset = Rotation2d.fromRadians(zeroOffset.in(Units.Radians));
    this.simDrivePosition = 0.0;
    this.simModulePosition = new SwerveModulePosition();
    this.desiredState = new SwerveModuleState(Units.MetersPerSecond.of(0.0), this.zeroOffset.plus(this.location.getLockPosition()));
    this.autoLockTime = MathUtil.clamp(autoLockTime.in(Units.Milliseconds), 0.0, CommonConstants.SwerveConstants.MAX_AUTO_LOCK_TIME * 1000);
    this.previousRotatePosition = this.zeroOffset.plus(this.location.getLockPosition());
    this.autoLockTimer = Instant.now();

    Logger.recordOutput(driveMotor.getID().name + CommonConstants.LogConstants.MAX_LINEAR_VELOCITY_LOG_ENTRY, DRIVE_MAX_LINEAR_SPEED);

    //Config Drive Motor 
    EXECUTOR_SERVICE.submit(() -> configDrive(driveWheel, motorOrientation, drivePID));

    // Reset the Drive Encoder
    EXECUTOR_SERVICE.submit(this::resetDriveEncoder);

    //Config Rotate Motor
    EXECUTOR_SERVICE.submit(() -> configRotate(motorOrientation, encoderOrientation, rotatePID));

    // Add callbacks to PurpleManager
    PurpleManager.addCallback(this::periodic);
    PurpleManager.addCallbackSim(this::simulationPeriodic);

    Runtime.getRuntime().addShutdownHook(new Thread(this::shutdownExecutorService));
  }

  public void configDrive(DriveWheel driveWheel, SwerveModule.MountOrientation motorOrientation,  PIDConstants drivePID) {
    SparkBaseConfig driveMotorConfig;
    
    // Configure the drive motor
    driveMotorConfig = driveMotor.getKind().equals(MotorKind.NEO_VORTEX) ? new SparkFlexConfig() : new SparkMaxConfig();

    // Set drive encoder config
    double m_driveConversionFactor = driveWheel.diameter.in(Units.Meters) * Math.PI / super.getGearRatio().getDriveRatio();
    driveMotorConfig.encoder.positionConversionFactor(m_driveConversionFactor);
    driveMotorConfig.encoder.velocityConversionFactor(m_driveConversionFactor / 60);

    // Invert drive motor if necessary
    driveMotorConfig.inverted(motorOrientation.equals(MountOrientation.INVERTED));

    // Set sensor to use for closed loop control
    driveMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // Set gains for drive PID
    driveMotorConfig.closedLoop.pid(
      drivePID.kP,
      drivePID.kI,
      drivePID.kD
    );

    // Set drive motor to coast
    driveMotorConfig.idleMode(IdleMode.kCoast);

    // Set current limits
    driveMotorConfig.smartCurrentLimit(CommonConstants.SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    // Set status frame rates
    driveMotorConfig.signals.primaryEncoderPositionPeriodMs(23);
    driveMotorConfig.signals.primaryEncoderVelocityPeriodMs(20);
    driveMotorConfig.signals.absoluteEncoderPositionPeriodMs(20);
    driveMotorConfig.signals.absoluteEncoderVelocityPeriodMs(20);
    driveMotorConfig.signals.analogPositionPeriodMs(20);
    driveMotorConfig.signals.analogVelocityPeriodMs(20);
    driveMotorConfig.signals.limitsPeriodMs(10);

    // Configure the drive motor with the desired config
    driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}

public void configRotate(SwerveModule.MountOrientation motorOrientation, SwerveModule.MountOrientation encoderOrientation, PIDConstants rotatePID) {
  SparkBaseConfig rotateMotorConfig;

  // Configure the rotate motor
  rotateMotorConfig = rotateMotor.getKind().equals(MotorKind.NEO_VORTEX) ? new SparkFlexConfig() : new SparkMaxConfig();

  // Set rotate encoder config
  double m_rotateConversionFactor = 6.28318530718;
  rotateMotorConfig.absoluteEncoder.positionConversionFactor(m_rotateConversionFactor);
  rotateMotorConfig.absoluteEncoder.velocityConversionFactor(m_rotateConversionFactor / 60);
  rotateMotorConfig.absoluteEncoder.inverted(!encoderOrientation.equals(motorOrientation));

  // Set sensor to use for closed loop control
  rotateMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

  // Set gains for rotate PID and enable wrapping
  rotateMotorConfig.closedLoop.pid(rotatePID.kP, rotatePID.kI, rotatePID.kD);
  rotateMotorConfig.closedLoop.positionWrappingEnabled(true);
  rotateMotorConfig.closedLoop.positionWrappingInputRange(0.0, m_rotateConversionFactor);

  // Set rotate motor to brake
  rotateMotorConfig.idleMode(IdleMode.kBrake);

  // Set current limits
  rotateMotorConfig.smartCurrentLimit(CommonConstants.SwerveConstants.ROTATE_MOTOR_CURRENT_LIMIT);

  // Set status frame rates
  rotateMotorConfig.signals.primaryEncoderPositionPeriodMs(23);
  rotateMotorConfig.signals.primaryEncoderVelocityPeriodMs(20);
  rotateMotorConfig.signals.absoluteEncoderPositionPeriodMs(20);
  rotateMotorConfig.signals.absoluteEncoderVelocityPeriodMs(20);
  rotateMotorConfig.signals.analogPositionPeriodMs(20);
  rotateMotorConfig.signals.analogVelocityPeriodMs(20);
  rotateMotorConfig.signals.limitsPeriodMs(10);

  // Configure the rotate motor with the desired config
  rotateMotor.configure(rotateMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
}


  private void shutdownExecutorService() {
    try {
        EXECUTOR_SERVICE.shutdown();
        if (!EXECUTOR_SERVICE.awaitTermination(60, TimeUnit.SECONDS)) {
          EXECUTOR_SERVICE.shutdownNow();
        }
    } catch (InterruptedException e) {
      EXECUTOR_SERVICE.shutdownNow();
    }
  }

  /**
   * Update position in simulation
   */
  void updateSimPosition() {
    simDrivePosition += desiredState.speedMetersPerSecond * GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds);
    synchronized (driveMotor.getInputs()) {
      driveMotor.getInputs().encoderPosition = simDrivePosition;
      driveMotor.getInputs().encoderVelocity = desiredState.speedMetersPerSecond;
      synchronized (rotateMotor.getInputs()) {
        rotateMotor.getInputs().absoluteEncoderPosition = desiredState.angle.getRadians();
        simModulePosition = new SwerveModulePosition(simDrivePosition, desiredState.angle);
      }
    }
  }

  /**
   * Call this method periodically
   */
  private void periodic() {
    super.logOutputs();
    Logger.recordOutput(rotateMotor.getID().name + CommonConstants.LogConstants.ROTATE_ERROR_LOG_ENTRY, desiredState.angle.minus(Rotation2d.fromRadians(rotateMotor.getInputs().absoluteEncoderPosition)));
  }

 /**
   * Call this method periodically in simulation
   */
  private void simulationPeriodic() {
    var vbus = Units.Volts.of(RobotController.getBatteryVoltage());
    driveMotor.getSim().enable();
    rotateMotor.getSim().enable();

    driveMotor.getSim().iterate(moduleSim.getDriveMotorVelocity().in(Units.RPM), vbus.in(Units.Volts), GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds));
    rotateMotor.getSim().iterate(moduleSim.getRotateMotorVelocity().in(Units.RPM), vbus.in(Units.Volts), GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds));

    moduleSim.update(
      vbus.times(driveMotor.getSim().getAppliedOutput()),
      vbus.times(rotateMotor.getSim().getAppliedOutput())
    );

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(moduleSim.getTotalCurrentDraw().in(Units.Amps)));

    driveMotor.getSim().setMotorCurrent(moduleSim.getDriveMotorCurrentDraw().in(Units.Amps));
    rotateMotor.getSim().setMotorCurrent(moduleSim.getRotateMotorCurrentDraw().in(Units.Amps));

    updateSimPosition();
  }

  /**
   * Allow for adding swerve module as a sendable object for dashboard interactivity
   * @param builder the builder
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSafeState(this::lock);
    builder.setActuator(true);
    // Control drive velocity
    builder.addDoubleProperty(
      "Velocity",
      () -> desiredState.speedMetersPerSecond,
      (value) -> set(new SwerveModuleState(Units.MetersPerSecond.of(value), desiredState.angle))
    );
    // Control rotation
    builder.addDoubleProperty(
      "Orientation",
      () -> desiredState.angle.getRadians(),
      (value) -> set(new SwerveModuleState(desiredState.speedMetersPerSecond, Rotation2d.fromRadians(value)))
    );
    // Configure drive kS
    builder.addDoubleProperty(
      "Drive kS",
      () -> driveFF.getKs(),
      (value) -> {
        driveFF = new SimpleMotorFeedforward(value, driveFF.getKv(), driveFF.getKa());
      }
    );
    // Configure drive kV
    builder.addDoubleProperty(
      "Drive kV",
      () -> driveFF.getKv(),
      (value) -> {
        driveFF = new SimpleMotorFeedforward(driveFF.getKs(), value, driveFF.getKa());
      }
    );
    // Configure drive kA
    builder.addDoubleProperty(
      "Drive kA",
      () -> driveFF.getKa(),
      (value) -> {
        driveFF = new SimpleMotorFeedforward(driveFF.getKs(), driveFF.getKv(), value);
      }
    );
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    driveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void set(SwerveModuleState state) {
    // Auto lock modules if auto lock enabled, speed not requested, and time has elapsed
    if (super.isAutoLockEnabled() && state.speedMetersPerSecond < CommonConstants.SwerveConstants.EPSILON) {
      state.speedMetersPerSecond = 0.0;
      // Time's up, lock now...
      if (Duration.between(autoLockTimer, Instant.now()).toMillis() > autoLockTime)
        state.angle = location.getLockPosition();
      // Waiting to lock...
      else state.angle = previousRotatePosition.minus(zeroOffset);
    } else {
      // Not locking this loop, restart timer...
      autoLockTimer = Instant.now();
    }

    // Save previous state
    var oldState = desiredState;

    // Get desired state
    desiredState = super.getDesiredState(state, Rotation2d.fromRadians(rotateMotor.getInputs().absoluteEncoderPosition));

    // Set rotate motor position
    rotateMotor.set(desiredState.angle.getRadians(), ControlType.kPosition);

    // Calculate drive FF
    var driveFF = this.driveFF.calculateWithVelocities(
      oldState.speedMetersPerSecond,
      desiredState.speedMetersPerSecond
    );

    // Set drive motor speed
    driveMotor.set(
      desiredState.speedMetersPerSecond, ControlType.kVelocity,
      driveFF, ArbFFUnits.kVoltage
    );

    // Save rotate position
    previousRotatePosition = desiredState.angle;

    // Increment odometer
    super.incrementOdometer(Math.abs(desiredState.speedMetersPerSecond) * GlobalConstants.ROBOT_LOOP_HZ.asPeriod().in(Units.Seconds));
  }

  @Override
  public LinearVelocity getDriveVelocity() {
    return Units.MetersPerSecond.of(driveMotor.getInputs().encoderVelocity);
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveVelocity(),
      Rotation2d.fromRadians(rotateMotor.getInputs().absoluteEncoderPosition).minus(zeroOffset)
    );
  }

  @Override
  public SwerveModulePosition getPosition() {
    if (RobotBase.isSimulation()) return simModulePosition;
    return new SwerveModulePosition(
      driveMotor.getInputs().encoderPosition,
      Rotation2d.fromRadians(rotateMotor.getInputs().absoluteEncoderPosition).minus(zeroOffset)
    );
  }

  @Override
  public void resetDriveEncoder() {
    driveMotor.resetEncoder();
    simDrivePosition = 0.0;
    simModulePosition = new SwerveModulePosition(simDrivePosition, previousRotatePosition);
  }

  @Override
  public void stop() {
    rotateMotor.stopMotor();
    driveMotor.stopMotor();
  }

  @Override
  public void close() {
    driveMotor.close();
    rotateMotor.close();
  }

  @Override
  public void setDriveSysID(Voltage volts) {
    throw new UnsupportedOperationException("Unimplemented method 'setDriveSysID'");
  }

  @Override
  public void setRotateSysID(Voltage volts) {
    throw new UnsupportedOperationException("Unimplemented method 'setRotateSysID'");
  }

  @Override
  public Frequency getUpdateRate() {
    throw new UnsupportedOperationException("Unimplemented method 'getUpdateRate'");
  }
}
