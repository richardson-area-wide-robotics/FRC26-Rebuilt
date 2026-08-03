// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.common.subsystems.drive;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommonConstants.DriveConstants;
import frc.robot.CommonConstants.HIDConstants;
import frc.robot.CommonConstants.LogConstants;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.subsystems.vision.AssumedPoseSubsystem.ModulePositionSupplier;
import frc.robot.common.swerve.MAXSwerveModule;

import org.littletonrobotics.junction.Logger;

public class SwerveDriveSubsystem extends SubsystemBase implements ModulePositionSupplier {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  /**
   * Pose estimator for tracking robot pose.
   *
   * <p>This replaces the plain {@code SwerveDriveOdometry} that was here previously. The
   * estimator accepts vision measurements via {@link #addVisionMeasurement}, which bounds
   * dead-reckoning drift over a match. With no vision source attached it behaves
   * identically to raw odometry, so this is a safe drop-in.
   */
  private final SwerveDrivePoseEstimator m_poseEstimator;

  /**
   * A parallel, vision-free odometry track.
   *
   * <p>Exists solely as the baseline for calibration: comparing it against AprilTag ground
   * truth is what measures wheel-scale error and gyro drift. Fusing vision into the pose you
   * then use to measure odometry error would hide the very thing you are looking for.
   */
  private final SwerveDriveOdometry m_odometryOnly;

  /** Last chassis speeds we were asked to achieve, kept for logging and diagnostics. */
  private ChassisSpeeds m_commandedSpeeds = new ChassisSpeeds();

  /** Creates a new DriveSubsystem. */
  public SwerveDriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroRotation(),
        get(),
        new Pose2d(),
        // State standard deviations: trust the wheels/gyro fairly well.
        VecBuilder.fill(0.05, 0.05, Math.toRadians(2.0)),
        // Default vision standard deviations. VisionSubsystem overrides these per
        // measurement, so these only apply to the two-argument addVisionMeasurement.
        VecBuilder.fill(0.5, 0.5, Math.toRadians(15.0)));

    m_odometryOnly = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, getGyroRotation(), get(), new Pose2d());
  }

  @Override
  public void periodic() {
    // Update the pose estimate in the periodic block
    m_poseEstimator.update(getGyroRotation(), get());

    // Keep the vision-free baseline moving in lockstep, for calibration comparison.
    m_odometryOnly.update(getGyroRotation(), get());

    logTelemetry();
  }

  /**
   * Publishes everything needed to (a) confirm the drivetrain is behaving as commanded and
   * (b) tune the module PID and feedforward from a log file after the fact.
   */
  private void logTelemetry() {
    SwerveModuleState[] actual = getModuleStates();
    SwerveModuleState[] desired = getDesiredModuleStates();

    Logger.recordOutput(getName() + LogConstants.POSE_LOG_ENTRY, getPose());
    // Logged side by side so the divergence is visible directly in AdvantageScope.
    Logger.recordOutput(getName() + "/OdometryOnlyPose", getOdometryOnlyPose());
    Logger.recordOutput(getName() + "/VisionCorrectionMeters",
        getPose().getTranslation().getDistance(getOdometryOnlyPose().getTranslation()));
    Logger.recordOutput(getName() + LogConstants.ACTUAL_SWERVE_STATE_LOG_ENTRY, actual);
    Logger.recordOutput(getName() + LogConstants.DESIRED_SWERVE_STATE_LOG_ENTRY, desired);
    Logger.recordOutput(getName() + LogConstants.MAX_LINEAR_VELOCITY_LOG_ENTRY,
        DriveConstants.kMaxSpeedMetersPerSecond);

    // Per-module velocity error is the single most useful signal for tuning the drive
    // closed loop; angle error does the same job for the steering loop.
    double[] velocityError = new double[actual.length];
    double[] angleErrorDeg = new double[actual.length];
    for (int i = 0; i < actual.length; i++) {
      velocityError[i] = desired[i].speedMetersPerSecond - actual[i].speedMetersPerSecond;
      angleErrorDeg[i] = desired[i].angle.minus(actual[i].angle).getDegrees();
    }
    Logger.recordOutput(getName() + "/VelocityErrorMetersPerSec", velocityError);
    Logger.recordOutput(getName() + LogConstants.ROTATE_ERROR_LOG_ENTRY, angleErrorDeg);

    // Commanded vs measured chassis speeds: proves the kinematics and the whole
    // stick-to-wheel path are doing what we asked.
    ChassisSpeeds measured = getChassisSpeeds();
    Logger.recordOutput(getName() + "/CommandedChassisSpeeds", m_commandedSpeeds);
    Logger.recordOutput(getName() + "/MeasuredChassisSpeeds", measured);

    Logger.recordOutput(getName() + "/GyroAngleDeg", getHeading());
    Logger.recordOutput(getName() + "/GyroRateDegPerSec", getTurnRate());
    Logger.recordOutput(getName() + "/GyroConnected", m_gyro.isConnected());
  }

  /** Gyro rotation, honouring {@code kGyroReversed}. */
  private Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(
        m_gyro.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the pose estimate to the specified pose.
   *
   * @param pose The pose to which to set the estimate.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(getGyroRotation(), get(), pose);
    m_odometryOnly.resetPosition(getGyroRotation(), get(), pose);
  }

  /**
   * Fuses an externally-measured field-relative pose into the estimate.
   *
   * <p>Uses the estimator's default trust level. Prefer
   * {@link #addVisionMeasurement(Pose2d, double, Matrix)} where the measurement's quality is
   * known, which is normally the case with AprilTags.
   *
   * @param visionPose        Field-relative pose reported by the vision system.
   * @param timestampSeconds  FPGA timestamp the measurement corresponds to.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    m_poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
  }

  /**
   * Fuses a vision pose with an explicit per-measurement trust level.
   *
   * <p>A tag two metres away deserves far more weight than the same tag at six metres, and a
   * multi-tag solve more than a single ambiguous one. Passing the standard deviations per
   * measurement is what turns vision from a source of jitter into a source of accuracy.
   *
   * @param visionPose       Field-relative pose reported by the vision system.
   * @param timestampSeconds FPGA timestamp the measurement corresponds to.
   * @param stdDevs          Trust: x metres, y metres, theta radians. Larger means less trust.
   */
  public void addVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    m_poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
  }

  /**
   * The pose from wheels and gyro alone, with no vision ever fused in.
   *
   * <p>Maintained purely so it can be compared against tag-derived ground truth. The gap
   * between this and the AprilTag pose over a run is what {@code VisionCalibration} turns
   * into a wheel-diameter correction and a gyro error figure — you cannot measure odometry
   * drift using an estimate that has already been corrected by vision.
   *
   * @return the vision-free pose estimate.
   */
  public Pose2d getOdometryOnlyPose() {
    return m_odometryOnly.getPoseMeters();
  }

  /** @return the current translational speed of the robot, in m/s. */
  public double getChassisSpeedMetersPerSecond() {
    ChassisSpeeds speeds = getChassisSpeeds();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward), as a fraction of
   *                      {@code kMaxSpeedMetersPerSecond}.
   * @param ySpeed        Speed of the robot in the y direction (sideways), as a fraction of
   *                      {@code kMaxSpeedMetersPerSecond}.
   * @param rot           Angular rate of the robot, as a fraction of {@code kMaxAngularSpeed}.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    m_commandedSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeedDelivered, ySpeedDelivered, rotDelivered, getGyroRotation())
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(m_commandedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    drive(
        chassisSpeeds.vxMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeed,
        false);
  }

  /**
   * Returns a command that drives the robot from live joystick input.
   *
   * <p>The suppliers are polled every scheduler run. They must be suppliers, not plain
   * doubles: an earlier version of this method took {@code double} parameters, so the
   * lambda captured whatever the sticks read at construction time — zero — and the robot
   * could not be driven at all.
   *
   * @param xSpeed        Supplier of forward speed, −1..1.
   * @param ySpeed        Supplier of sideways speed, −1..1.
   * @param rot           Supplier of rotation rate, −1..1.
   * @param fieldRelative Whether x/y are field-relative.
   * @return A command suitable for use as the drivetrain's default command.
   */
  public Command driveCommand(
      DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, boolean fieldRelative) {
    return run(() -> drive(
        applyDeadband(xSpeed.getAsDouble()),
        applyDeadband(ySpeed.getAsDouble()),
        applyDeadband(rot.getAsDouble()),
        fieldRelative));
  }

  /**
   * Applies the configured controller deadband, rescaling the remaining travel so that the
   * output is still continuous from 0 to 1 just outside the deadband.
   *
   * @param value Raw axis value.
   * @return Deadbanded axis value.
   */
  public static double applyDeadband(double value) {
    return MathUtil.applyDeadband(value, HIDConstants.CONTROLLER_DEADBAND);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return getGyroRotation().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /** @return the measured state of each module, in FL, FR, RL, RR order. */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }

  /** @return the last commanded state of each module, in FL, FR, RL, RR order. */
  public SwerveModuleState[] getDesiredModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getDesiredState(),
      m_frontRight.getDesiredState(),
      m_rearLeft.getDesiredState(),
      m_rearRight.getDesiredState()
    };
  }

  /**
   * Returns the robot-relative chassis speeds as measured by the modules.
   *
   * <p>This previously called {@code toChassisSpeeds()} with no arguments, i.e. an empty
   * varargs array, so it never reflected the real module states. PathPlanner consumes this
   * as its velocity feedback, so autonomous path following was running against a bogus
   * measurement.
   *
   * @return Measured robot-relative chassis speeds.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void configureAutoBuilder() {
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(14.0, 0.0, 0.1), // Translation PID constants
                    new PIDConstants(2.1, 0.0, 0.2) // Rotation PID constants
            ),
            RobotUtils.getRobotConfig(), // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
              var alliance = DriverStation.getAlliance();
              return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public SwerveModulePosition[] get() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }
}
