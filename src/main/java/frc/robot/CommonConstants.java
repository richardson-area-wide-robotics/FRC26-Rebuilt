// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/**
 * The CommonConstants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class CommonConstants {

  /**
   * CommonConstants for controllers
   * <br>
   * {@link HIDConstants#PRIMARY_CONTROLLER_PORT} is for the driver,
   * <br> <br>
   * {@link HIDConstants#SECONDARY_CONTROLLER_PORT} is for the operator
   */
  public static class HIDConstants {

    
  public static final boolean SILENCE_NO_CONTROLLER_WARNING = true;
  public static final int PRIMARY_CONTROLLER_PORT = 0;
  public static final int SECONDARY_CONTROLLER_PORT = 1;

  /**
   * Fraction of stick travel treated as zero. Applied in
   * {@link frc.robot.common.subsystems.drive.SwerveDriveSubsystem#driveCommand}.
   *
   * <p>This was previously 0.6, which would have discarded 60% of stick travel — but it
   * was never actually applied anywhere, so the robot saw raw stick values. Now that the
   * deadband is really wired in, it is set to a normal value.
   */
  public static final double CONTROLLER_DEADBAND = 0.08;

  public static final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(
    PRIMARY_CONTROLLER_PORT);
  public static final CommandXboxController OPERATOR_CONTROLLER = new CommandXboxController(
    SECONDARY_CONTROLLER_PORT);

  }

  public static class SwerveConstants {

    /**
     * In Amps, the max current a swerve Drive motor may draw.
     *
     * <p>This is now the single source of truth — {@link frc.robot.common.swerve.Configs}
     * reads it. It is 50 rather than the 60 previously declared here, because 50 is the
     * value that was actually being applied to the hardware; the old constant was dead.
     */
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 50;

    /** In Amps, the max current a swerve Rotate motor may draw. */
    public static final int ROTATE_MOTOR_CURRENT_LIMIT = 20;

  }

  /**
   * In Amps, the max current a superstructure (intake/feeder/shooter/climber) motor may
   * draw. Matches the value each subsystem constructor was already applying.
   */
  public static final int SUPERSTRUCTURE_CURRENT_LIMIT = 60;

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward.
    // NOTE: the divisor must stay a double literal. As `5676 / 60` this was integer
    // division, yielding 94.0 instead of 94.6 and biasing the drive feedforward.
    public static final double kDrivingMotorFreeSpeedRps = 5676 / 60.0;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = edu.wpi.first.math.util.Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 1;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 2;

    public static final boolean kGyroReversed = false;
  }

  public static class SmartDashboardConstants {
    public static final String SMARTDASHBOARD_AUTO_MODE = "Auto Mode";
  }

  public static class LogConstants {
    public static final String POSE_LOG_ENTRY = "/Pose";
    public static final String ACTUAL_SWERVE_STATE_LOG_ENTRY = "/ActualSwerveState";
    public static final String DESIRED_SWERVE_STATE_LOG_ENTRY = "/DesiredSwerveState";
    public static final String ROTATE_ERROR_LOG_ENTRY = "/RotateError";
    public static final String MAX_LINEAR_VELOCITY_LOG_ENTRY = "/MaxLinearVelocity";

  }

}