// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.common.subsystems.drive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.CommonConstants;
import frc.robot.common.components.hardware.SwerveHardware;
import frc.robot.common.components.hardware.SwerveHardwareParams;
import frc.robot.common.gyro.RAWRNavX2;
import frc.robot.common.subsystems.DashboardSubsystem;
import frc.robot.common.subsystems.vision.AssumedPoseSubsystem;
import lombok.Getter;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics;
import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity;
import org.lasarobotics.drive.RotatePIDController;
import org.lasarobotics.drive.ThrottleMap;
import org.lasarobotics.utils.PIDConstants;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // For dashboard integration
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.components.NamedAutoRegistry;
import frc.robot.common.components.RobotUtils;

/**
 * Drive Subsystem for Swerve Drive bots with 4 motors in each corner
 *
 * @author PurpleLib
 * @author Alan Trinh
 * @author Hudson Strub
 *
 * @since 2025
 */
public class SwerveDriveSubsystem extends DashboardSubsystem implements AutoCloseable {


  public final SwerveHardware DRIVETRAIN_HARDWARE;

  private final ThrottleMap THROTTLE_MAP;
  private final RotatePIDController ROTATE_PID_CONTROLLER;
  @Getter
  private final SwerveDriveKinematics KINEMATICS;

  public final LinearVelocity DRIVE_MAX_LINEAR_SPEED;
  public final LinearAcceleration DRIVE_AUTO_ACCELERATION;
  private final AdvancedSwerveKinematics ADVANCED_KINEMATICS;
  private final ProfiledPIDController AUTO_AIM_PID_CONTROLLER_FRONT;
  private final ProfiledPIDController AUTO_AIM_PID_CONTROLLER_BACK;
  private final PPHolonomicDriveController PATH_FOLLOWER_CONFIG;

  private ControlCentricity controlCentricity;
  private ChassisSpeeds desiredChassisSpeeds;
  private Pose2d previousPose;
  private Rotation2d currentHeading;
  public final Field2d FIELD;

  public final AssumedPoseSubsystem ASSUMED_POSE;


  /**
   * SwerveDriveSubsystem constructor for managing a swerve drivetrain.
   *
   * @param drivetrainHardwareParams Hardware devices required by drivetrain.
   * @param pidf PID constants for the drive system.
   * @param controlCentricity Control centricity configuration.
   * @param throttleInputCurve Spline function for throttle input.
   * @param turnInputCurve Spline function for turn input.
   * @param turnScalar Scalar for turn input (degrees).
   * @param deadband Deadband for controller input [+0.001, +0.2].
   * @param lookAhead Rotate PID lookahead, in number of loops.
   */
  public SwerveDriveSubsystem(SwerveHardwareParams drivetrainHardwareParams, PIDConstants pidf, ControlCentricity controlCentricity,
                              PolynomialSplineFunction throttleInputCurve, PolynomialSplineFunction turnInputCurve,
                              Angle turnScalar, Dimensionless deadband, Time lookAhead) {
     // Initialize subsystem name
        setSubsystem(this.getClass().getSimpleName());

        this.DRIVETRAIN_HARDWARE = drivetrainHardwareParams.initializeHardware();

        // Drivetrain constants
        DRIVE_MAX_LINEAR_SPEED = DRIVETRAIN_HARDWARE.lFrontModule().getMaxLinearVelocity();
        DRIVE_AUTO_ACCELERATION = DRIVE_MAX_LINEAR_SPEED
            .per(Units.Second)
            .minus(Units.MetersPerSecondPerSecond.of(1.0));

        // Input curves and controllers
        this.controlCentricity = controlCentricity;
        this.THROTTLE_MAP = new ThrottleMap(throttleInputCurve, DRIVE_MAX_LINEAR_SPEED, deadband);
        this.ROTATE_PID_CONTROLLER = new RotatePIDController(turnInputCurve, pidf, turnScalar, deadband, lookAhead);

        // Path follower configuration
        this.PATH_FOLLOWER_CONFIG = new PPHolonomicDriveController(
            new com.pathplanner.lib.config.PIDConstants(14.0, 0, 0.1),
            new com.pathplanner.lib.config.PIDConstants(2.1, 0, 0.2)
        );

        // Swerve drive kinematics and pose estimator
        KINEMATICS = new SwerveDriveKinematics(DRIVETRAIN_HARDWARE.getModuleCoordinates());

        ADVANCED_KINEMATICS = new AdvancedSwerveKinematics(DRIVETRAIN_HARDWARE.getModuleCoordinates());


        ASSUMED_POSE = new AssumedPoseSubsystem(
                (RAWRNavX2) DRIVETRAIN_HARDWARE.gyro(),
                getKINEMATICS(),
                DRIVETRAIN_HARDWARE::getModulePositions,
                new Transform3d(-0.27, 0.12, 0.6223, new Rotation3d(0, 0.087, 0)),
                "OV9281"
        );
      // Chassis speeds
      desiredChassisSpeeds = new ChassisSpeeds();

      // Field2d visualization
      FIELD = new Field2d();
      SmartDashboard.putData(FIELD);

      // Path logging for PathPlanner
      PathPlannerLogging.setLogActivePathCallback((poses) -> {
          if (poses.isEmpty()) return;
          Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
              poses,
              new TrajectoryConfig(DRIVE_MAX_LINEAR_SPEED, DRIVE_AUTO_ACCELERATION)
          );
          FIELD.getObject("currentPath").setTrajectory(trajectory);
      });

      this.AUTO_AIM_PID_CONTROLLER_FRONT = new ProfiledPIDController(10.0, 0.0, 0.5, new TrapezoidProfile.Constraints(2160.0, 4320.0), 0.02);
      this.AUTO_AIM_PID_CONTROLLER_FRONT.enableContinuousInput(-180.0, +180.0);
      this.AUTO_AIM_PID_CONTROLLER_FRONT.setTolerance(1.5);
      this.AUTO_AIM_PID_CONTROLLER_BACK = new ProfiledPIDController(10.0, 0.0, 0.5, new TrapezoidProfile.Constraints(2160.0, 4320.0), 0.02);
      this.AUTO_AIM_PID_CONTROLLER_BACK.enableContinuousInput(-180.0, +180.0);
      this.AUTO_AIM_PID_CONTROLLER_BACK.setTolerance(1.5);

      // Register @NamedAuto commands manually to improve boot-up speed
      NamedAutoRegistry.register(this);
}


  /**
   * Drive the robot
   *
   * @param xRequest         Desired X (forward) velocity
   * @param yRequest         Desired Y (sideways) velocity
   * @param rotateRequest    Desired rotate rate
   * @param controlCentricity Control centricity
   */
  private void drive(LinearVelocity xRequest,
                    LinearVelocity yRequest,
                    AngularVelocity rotateRequest,
                    ControlCentricity controlCentricity) {

      if (controlCentricity == null){
        controlCentricity = ControlCentricity.FIELD_CENTRIC;
      }

      // Get requested chassis speeds, correcting for second order kinematics
      desiredChassisSpeeds = AdvancedSwerveKinematics.correctForDynamics(
          new ChassisSpeeds(xRequest.unaryMinus(), yRequest.unaryMinus(), rotateRequest)
      );

      // Convert speeds to module states, correcting for 2nd order kinematics
      SwerveModuleState[] moduleStates = ADVANCED_KINEMATICS.toSwerveModuleStates(
          desiredChassisSpeeds,
              ASSUMED_POSE.getPose().getRotation(),
          controlCentricity
      );

      // Desaturate drive speeds
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DRIVE_MAX_LINEAR_SPEED);

      // Set modules to calculated states, applying traction control if enabled
      DRIVETRAIN_HARDWARE.setSwerveModules(moduleStates);
  }

  /**
   * Update robot pose
   */
  private void updatePose() {
    Pose2d currentPose = getPose();

    if (previousPose != null) {
      double dx = currentPose.getX() - previousPose.getX();
      double dy = currentPose.getY() - previousPose.getY();

      if (Math.abs(dx) > 1e-6 || Math.abs(dy) > 1e-6) {
        currentHeading = new Rotation2d(Math.atan2(dy, dx));
      }
    }

    // Update AFTER calculating delta
    previousPose = currentPose;
  }

  /**
   * Log SwerveDriveSubsystem outputs
   */
  private void logOutputs() {
    Logger.recordOutput(getName() + CommonConstants.LogConstants.POSE_LOG_ENTRY, getPose());
    Logger.recordOutput(getName() + CommonConstants.LogConstants.ACTUAL_SWERVE_STATE_LOG_ENTRY,   DRIVETRAIN_HARDWARE.getModuleStates());
  }

  public Command lockCommand() {
    return runOnce(DRIVETRAIN_HARDWARE::lock);
  }

  /**
   * SmartDashboard indicators
   */
  private void smartDashboard() {
    // Update the robot pose normally
    FIELD.setRobotPose(getPose());

    // Get the robot's current position
    Translation2d robotPos = getPose().getTranslation();

    // Use currentHeading if you want motion-based heading, else gyro heading
    Rotation2d heading = currentHeading != null ? currentHeading : ASSUMED_POSE.getPose().getRotation();

    // Create a small square offset from robot to indicate heading
    double squareSize = 0.5; // meters, adjust as needed
    Translation2d squarePos = robotPos.plus(
            new Translation2d(Math.cos(heading.getRadians()), Math.sin(heading.getRadians())).times(squareSize)
    );

    // Draw a red square at that position
    FIELD.getObject("HeadingSquare").setPose(new Pose2d(squarePos, heading));

    //FIELD.getObject("REALBot").setPose(ASSUMED_POSE.getPose());

    // Field-centric indicator
    SmartDashboard.putBoolean("FC", controlCentricity.equals(ControlCentricity.FIELD_CENTRIC));
  }


  /**
   * Aim robot at a desired point on the field
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param rotateRequest Desired rotate speed (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param point Target point, pass in null to signify invalid point
   * @param controlCentricity True to point back of robot to target
   * @param velocityCorrection True to compensate for robot's own velocity
   */
  private void aimAtPoint(ControlCentricity controlCentricity, double xRequest, double yRequest, double rotateRequest, Translation2d point, boolean reversed, boolean velocityCorrection) {
    // Calculate desired robot velocity
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);
    LinearVelocity velocityOutput = THROTTLE_MAP.throttleLookup(moveRequest);

    // Drive normally and return if invalid point
    if (point == null) {
      AngularVelocity rotateOutput = ROTATE_PID_CONTROLLER.calculate(DRIVETRAIN_HARDWARE.gyro().getYaw(), DRIVETRAIN_HARDWARE.gyro().getYawRate(), rotateRequest).unaryMinus();
      drive(
        velocityOutput.unaryMinus().times(Math.cos(moveDirection)),
        velocityOutput.unaryMinus().times(Math.sin(moveDirection)),
        rotateOutput,
        controlCentricity
      );
      return;
    }

    // Adjust point
    point = point.plus(CommonConstants.DriveConstants.AIM_OFFSET);
    // Get current pose
    Pose2d currentPose = getPose();
    // Angle to target point
    Rotation2d targetAngle = new Rotation2d(point.getX() - currentPose.getX(), point.getY() - currentPose.getY());
    // Movement vector of robot
    Vector2D robotVector = new Vector2D(velocityOutput.times(currentHeading.getCos()).magnitude(), velocityOutput.times(currentHeading.getSin()).magnitude());
    // Aim point
    Translation2d aimPoint = point.minus(new Translation2d(robotVector.getX(), robotVector.getY()));
    // Vector from robot to target
    Vector2D targetVector = new Vector2D(currentPose.getTranslation().getDistance(point) * targetAngle.getCos(), currentPose.getTranslation().getDistance(point) * targetAngle.getSin());
    // Parallel component of robot's motion to target vector
    Vector2D parallelRobotVector = targetVector.scalarMultiply(robotVector.dotProduct(targetVector) / targetVector.getNormSq());
    // Perpendicular component of robot's motion to target vector
    Vector2D perpendicularRobotVector = robotVector.subtract(parallelRobotVector).scalarMultiply(velocityCorrection ?CommonConstants.DriveConstants. AIM_VELOCITY_COMPENSATION_FUDGE_FACTOR : 0.0);
    // Adjust aim point using calculated vector
    Translation2d adjustedPoint = point.minus(new Translation2d(perpendicularRobotVector.getX(), perpendicularRobotVector.getY()));
    // Calculate new angle using adjusted point
    Rotation2d adjustedAngle = new Rotation2d(adjustedPoint.getX() - currentPose.getX(), adjustedPoint.getY() - currentPose.getY());
    // Calculate necessary rotate rate
    double rotateOutput = reversed
      ? AUTO_AIM_PID_CONTROLLER_BACK.calculate(currentPose.getRotation().plus(Rotation2d.fromRadians(Math.PI)).getDegrees(), adjustedAngle.getDegrees())
      : AUTO_AIM_PID_CONTROLLER_FRONT.calculate(currentPose.getRotation().getDegrees(), adjustedAngle.getDegrees());

    // Log aim point
    Logger.recordOutput(getName() + "/AimPoint", new Pose2d(aimPoint, new Rotation2d()));
    double aimError = currentPose.getRotation().getDegrees() - adjustedAngle.getDegrees();
    Logger.recordOutput(getName() + "/AimError", Math.copySign(((180 - Math.abs(aimError)) % 180), aimError));

    // Drive robot accordingly
    drive(
      velocityOutput.unaryMinus().times(Math.cos(moveDirection)),
      velocityOutput.unaryMinus().times(Math.sin(moveDirection)),
      Units.DegreesPerSecond.of(rotateOutput),
      controlCentricity);
  }

  /**
   * Configure the auto builder
   */
  public void configureAutoBuilder() {
    AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            this::autoDrive,
            PATH_FOLLOWER_CONFIG,
            RobotUtils.getRobotConfig(),
            () -> {
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this
    );
  }

  /**
   * Call this repeatedly to drive using PID during teleoperation
   * @param xRequest Desired X axis (forward) speed [-1.0, +1.0]
   * @param yRequest Desired Y axis (sideways) speed [-1.0, +1.0]
   * @param rotateRequest Desired rotate speed [-1.0, +1.0]
   */
  private void teleopPID(double xRequest, double yRequest, double rotateRequest) {
    // Calculate move request and direction
    double moveRequest = Math.hypot(xRequest, yRequest);
    double moveDirection = Math.atan2(yRequest, xRequest);

    // Get throttle and rotate output
    LinearVelocity velocityOutput = THROTTLE_MAP.throttleLookup(moveRequest);
    AngularVelocity rotateOutput = ROTATE_PID_CONTROLLER.calculate(DRIVETRAIN_HARDWARE.gyro().getYaw(), DRIVETRAIN_HARDWARE.gyro().getYawRate(), rotateRequest).unaryMinus();

    // Drive robot
    drive(
      velocityOutput.unaryMinus().times(Math.cos(moveDirection)),
      velocityOutput.unaryMinus().times(Math.sin(moveDirection)),
      rotateOutput,
      controlCentricity
    );
  }

  /**
   * Reset pose estimator
   * @param pose Pose to set robot to
   */
  public void resetPose(Pose2d pose) {
    ASSUMED_POSE.resetPose(pose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DRIVETRAIN_HARDWARE.gyro().updateInputs();


    // Filter inertial velocity
//    DRIVETRAIN_HARDWARE.gyro().getVelocityX() = Units.MetersPerSecond.of(
//      X_VELOCITY_FILTER.calculate(DRIVETRAIN_HARDWARE.gyro().getInputs().velocityX.in(Units.MetersPerSecond)
//    )).mutableCopy();
//    DRIVETRAIN_HARDWARE.gyro().getVelocityY() = Units.MetersPerSecond.of(
//      Y_VELOCITY_FILTER.calculate(DRIVETRAIN_HARDWARE.gyro().getInputs().velocityY.in(Units.MetersPerSecond)
//
//    )).mutableCopy();


    updatePose();
    smartDashboard();
    logOutputs();
  }

  @Override
  public void simulationPeriodic() {

    updatePose();
    smartDashboard();
    logOutputs();
  }

  /**
   * Call this repeatedly to drive during autonomous
   * @param speeds Calculated swerve module states
   */
  public void autoDrive(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {

    // DO NOT negate (auto speeds are already robot-relative)
    desiredChassisSpeeds =
            AdvancedSwerveKinematics.correctForDynamics(speeds);

    // Use ROBOT_CENTRIC for auto
    SwerveModuleState[] moduleStates =
            ADVANCED_KINEMATICS.toSwerveModuleStates(
                    desiredChassisSpeeds,
                    ASSUMED_POSE.getPose().getRotation(),
                    AdvancedSwerveKinematics.ControlCentricity.ROBOT_CENTRIC
            );

    // Same safety clamp as teleop
    SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates,
            DRIVE_MAX_LINEAR_SPEED
    );

    // Hardware only supports states (no feedforward yet)
    DRIVETRAIN_HARDWARE.setSwerveModules(moduleStates);
  }
  /**
   * Toggles between field centric and robot centric drive control
   */
  private void toggleControlCentricity() {
    if (controlCentricity == ControlCentricity.FIELD_CENTRIC) {
      this.controlCentricity = ControlCentricity.ROBOT_CENTRIC;
    } else {
      this.controlCentricity = ControlCentricity.FIELD_CENTRIC;
    }
  }

  /**
   * Aim robot at desired point on the field, while strafing
   * @param xRequestSupplier X axis speed supplier [-1.0, +1.0]
   * @param yRequestSupplier Y-axis speed supplier [-1.0, +1.0]
   * @param rotateRequestSupplier Rotate speed supplier (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param pointSupplier Desired point supplier
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim at point while strafing
   */
  public Command aimAtPointCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier,
                                   Supplier<Translation2d> pointSupplier, boolean reversed, boolean velocityCorrection) {
    return runEnd(() -> aimAtPoint(
            controlCentricity,
      xRequestSupplier.getAsDouble(),
      yRequestSupplier.getAsDouble(),
      rotateRequestSupplier.getAsDouble(),
      pointSupplier.get(),
      reversed,
      velocityCorrection
    ),
            this::resetRotatePID
    );
  }

  /**
   * Aim robot at desired point on the field, while strafing
   * @param xRequestSupplier X axis speed supplier [-1.0, +1.0]
   * @param yRequestSupplier Y axis speed supplier [-1.0, +1.0]
   * @param rotateRequestSupplier Rotate speed supplier (ONLY USED IF POINT IS NULL) [-1.0, +1.0]
   * @param point Desired point
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim at point while strafing
   */
  public Command aimAtPointCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier,
                                   Translation2d point, boolean reversed, boolean velocityCorrection) {
    return aimAtPointCommand(xRequestSupplier, yRequestSupplier, rotateRequestSupplier, () -> point, reversed, velocityCorrection);
  }

  /**
   * Aim robot at desired point on the field
   * @param point Desired point
   * @param reversed True to point rear of robot toward point
   * @param velocityCorrection True to compensate for robot's own velocity
   * @return Command that will aim robot at point while strafing
   */
  public Command aimAtPointCommand(Translation2d point, boolean reversed, boolean velocityCorrection) {
    return aimAtPointCommand(() -> 0.0, () -> 0.0, () -> 0.0, () -> point, reversed, velocityCorrection);
  }


  /**
   * Drive the robot
   * @param xRequestSupplier X axis speed supplier
   * @param yRequestSupplier Y axis speed supplier
   * @param rotateRequestSupplier Rotate speed supplier
   * @return Command that will drive robot
   */
  public Command driveCommand(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier) {
    return run(() -> teleopPID(xRequestSupplier.getAsDouble(), yRequestSupplier.getAsDouble(), rotateRequestSupplier.getAsDouble()));
  }

  /**
   * Stop robot
   * @return Command to stop robot
   */
  public Command stopCommand() {
    return runOnce(() -> {
      DRIVETRAIN_HARDWARE.stop();
      resetRotatePID();
    });
  }

  /**
   * Toggle traction control
   * @return Command to toggle traction control
   */
  public Command toggleTractionControlCommand() {
    return runOnce(() -> DRIVETRAIN_HARDWARE.toggleTractionControl());  }

  /**
   * Toggles between field and robot oriented drive control
   * @return Command to toggle control centricity between robot and field centric drive control
   */
  public Command toggleCentricityCommand() {
    return runOnce(this::toggleControlCentricity);
  }

  /**
   * Enable traction control
   * @return Command to enable traction control
   */
  public Command enableTractionControlCommand() {
    return runOnce(DRIVETRAIN_HARDWARE::enableTractionControl);  }

  /**
   * Disable traction control
   * @return Command to disable traction control
   */
  public Command disableTractionControlCommand() {
    return runOnce(DRIVETRAIN_HARDWARE::disableTractionControl);  }

  /**
   * Reset pose estimator
   * @param poseSupplier Pose supplier
   * @return Command to reset pose
   */
  public Command resetPoseCommand(Supplier<Pose2d> poseSupplier) {
    return runOnce(() -> resetPose(poseSupplier.get()));
  }

  /**
   * Aim robot at desired point on the field
   *
   * @return Command to aim a point on the field in robot centric mode
   */
  public Command aimAtPointRobotCentric(DoubleSupplier xRequestSupplier, DoubleSupplier yRequestSupplier, DoubleSupplier rotateRequestSupplier,
                                        Supplier<Translation2d> pointSupplier, boolean reversed, boolean velocityCorrection) {
    return runEnd(() ->
      aimAtPoint(
        ControlCentricity.ROBOT_CENTRIC,
        xRequestSupplier.getAsDouble(),
        yRequestSupplier.getAsDouble(),
        rotateRequestSupplier.getAsDouble(),
        pointSupplier.get(),
        reversed,
        velocityCorrection
      ),
            this::resetRotatePID
    );

  }

  /**
   * Reset SwerveDriveSubsystem turn PID
   */
  public void resetRotatePID() {
    ROTATE_PID_CONTROLLER.setSetpoint(DRIVETRAIN_HARDWARE.gyro().getYaw());
    ROTATE_PID_CONTROLLER.reset();
  }

    /**
   * Get constraints for path following
   * @return Path following constraints
   */
  public PathConstraints getPathConstraints() {
    return new PathConstraints(
      3.0,
      1.0,
      CommonConstants.DriveConstants.DRIVE_ROTATE_VELOCITY.in(Units.RadiansPerSecond),
      CommonConstants.DriveConstants.DRIVE_ROTATE_ACCELERATION.magnitude()
    );
  }

  /**
   * Get robot relative speeds
   * @return Robot relative speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return KINEMATICS.toChassisSpeeds(DRIVETRAIN_HARDWARE.getModuleStates());
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
            getChassisSpeeds(),
            getPose().getRotation()
    );
  }

  /**
   * Get estimated robot pose
   * @return Currently estimated robot pose
   */
  public Pose2d getPose() {
    return ASSUMED_POSE.getPose();
  }

  /**
   * Get whether robot is tipping over
   * @return True if robot is tipping
   */
  public boolean isTipping() {
    return Math.abs(DRIVETRAIN_HARDWARE.gyro().getPitch().in(Units.Degrees)) > CommonConstants.DriveConstants.TIP_THRESHOLD ||
           Math.abs(DRIVETRAIN_HARDWARE.gyro().getRoll().in(Units.Degrees)) > CommonConstants.DriveConstants.TIP_THRESHOLD;
  }

  /**
   * Get whether robot is nearly balanced
   * @return True if robot is (nearly) balanced
   */
  public boolean isBalanced() {
    return Math.abs(DRIVETRAIN_HARDWARE.gyro().getPitch().in(Units.Degrees)) < CommonConstants.DriveConstants.BALANCED_THRESHOLD &&
           Math.abs(DRIVETRAIN_HARDWARE.gyro().getRoll().in(Units.Degrees)) < CommonConstants.DriveConstants.BALANCED_THRESHOLD;
  }

  /**
   * Get if robot is aimed at desired target
   * @return True if aimed
   */
  public boolean isAimed() {
    return (AUTO_AIM_PID_CONTROLLER_FRONT.atGoal() || AUTO_AIM_PID_CONTROLLER_BACK.atGoal()) && DRIVETRAIN_HARDWARE.gyro().getYawRate().lt(CommonConstants.DriveConstants.AIM_VELOCITY_THRESHOLD);
  }

  /**
   * Get inertial velocity of robot
   * @return Inertial velocity of robot in m/s
   */
  public LinearVelocity getInertialVelocity() {
    return Units.MetersPerSecond.of(
      Math.hypot(DRIVETRAIN_HARDWARE.gyro().getVelocityX().in(Units.MetersPerSecond), DRIVETRAIN_HARDWARE.gyro().getVelocityY().in(Units.MetersPerSecond))
    );
  }

  @Override
  public void close() {
    DRIVETRAIN_HARDWARE.close();
  }
}