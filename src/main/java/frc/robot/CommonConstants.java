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

  /**
   * The bus voltage this robot actually sees.
   *
   * <p>Recorded because <b>12 V is not a safe assumption</b> and several thresholds were quietly built
   * on one. Observed on this robot: <b>6 V to 16 V, and mostly 10 V to 14 V.</b>
   *
   * <p>That range matters more than it looks. A duty-cycle command is a fraction of whatever the bus
   * happens to be, so the same command produces roughly <b>half the torque at 6 V that it does at
   * 12 V</b>. Anything expressed as duty cycle silently rescales itself as the pack drains; anything
   * expressed in volts does not, which is why the calibrations command volts.
   *
   * <p>Speed scales the same way, which is why {@code MotorLoadMonitor} compensates its expected speed
   * for bus voltage before comparing anything to it.
   */
  public static final class BatteryConstants {

    /** Nominal pack voltage. The reference every derived feedforward is expressed against. */
    public static final double NOMINAL_VOLTS = 12.0;

    /** Bottom of the usual range in normal operation. */
    public static final double TYPICAL_MIN_VOLTS = 10.0;

    /** Top of the usual range, i.e. a fresh pack off the charger. */
    public static final double TYPICAL_MAX_VOLTS = 14.0;

    /**
     * Lowest voltage seen at all, under the hardest load.
     *
     * <p>Genuinely low: the roboRIO's own brownout threshold is around 6.8 V and the SPARKs are not far
     * behind, so this is the edge of the robot working at all rather than merely working poorly.
     */
    public static final double ABSOLUTE_MIN_VOLTS = 6.0;

    /** Highest seen. */
    public static final double ABSOLUTE_MAX_VOLTS = 16.0;

    /**
     * Below this, a sag is worth reporting rather than expected.
     *
     * <p>8.5 rather than the 9.5 originally used. 9.5 sits <em>inside</em> the normal 10-to-14 band once
     * load is applied, so it flagged healthy behaviour as a battery fault — a false positive that sends
     * someone to the charger instead of at the real problem. 8.5 is clearly below anything this robot
     * does in normal operation and still well above brownout.
     */
    public static final double CONCERNING_SAG_VOLTS = 8.5;

    private BatteryConstants() {
    }
  }

  public static final class ModuleConstants {
    /**
     * Teeth on the driving pinion. <b>14T, confirmed.</b>
     *
     * <p>REV ships 12T, 13T and 14T. A larger pinion drives faster.
     */
    public static final int kDrivingMotorPinionTeeth = 14;

    /**
     * Teeth on the first-stage spur gear. <b>21T, confirmed — REV MAXSwerve "Extra High 1".</b>
     *
     * <p>This was hard-coded as 22 inside the reduction expression, which is WPILib's MAXSwerve
     * template default and gives the "High" 4.71:1 ratio. This robot runs the <b>Extra High 1</b>
     * gearing (REV-21-3008): 14T pinion against a <b>21T</b> spur, for exactly <b>4.50:1</b>.
     *
     * <p>Named rather than buried in the expression because this is the <b>third</b> time a WPILib
     * template default has turned out not to describe this robot. First the drive free speed was the
     * NEO's 5676 RPM rather than the Vortex's 6784. Then the module spacing was the frame perimeter
     * rather than the module axes. Now the spur gear. Each looked plausible precisely because it was a
     * real number for a real MAXSwerve — just not this one.
     *
     * <p><b>Combined, the drivetrain model was 25.2% wrong.</b> As found, the wrong free speed and the
     * wrong spur gear together gave 4.8037 m/s and a kV of 2.4981. Correct is 6.0149 m/s and 1.9951.
     */
    public static final int kDrivingMotorSpurTeeth = 21;

    /**
     * Free speed of the <b>drive</b> motor, in revolutions per second.
     *
     * <p>This robot's drivetrain runs <b>NEO Vortex</b> on SPARK Flex, free speed
     * <b>6784 RPM</b> per REV's datasheet. The turning modules run NEO 550 on SPARK MAX, and
     * the superstructure is a mix — but only the drive motor's free speed belongs here.
     *
     * <p>It previously read {@code 5676 / 60}. That is the free speed of the <b>NEO 2.0</b>
     * (and of the NEO 1.1 — both are 5676 RPM), and it is the default in WPILib's MAXSwerve
     * template. The trap is that this robot <em>does</em> carry NEO 2.0s — the intake deploy and
     * the spindexer — so 5676 is a real figure for a real motor here, just not for this shaft.
     * Nothing about it looks wrong on inspection. So it looked plausible while being 19.5% wrong
     * for the drivetrain, and the error went straight into the velocity feedforward:
     * {@code kV = 12 V / kDriveWheelFreeSpeedRps} came out at 2.50 instead of 2.09, so the
     * feedforward over-commanded voltage on every drive request and the closed loop had to fight
     * it continuously.
     *
     * <p>The divisor must also stay a double literal: as {@code 5676 / 60} this was integer
     * division, discarding the fraction on top of naming the wrong motor.
     *
     * <p><b>The spur gear was wrong too</b>, see {@link #kDrivingMotorSpurTeeth}. Together the two
     * errors put the drivetrain model out by <b>25.2%</b> on kV, not the 19.5% the motor alone
     * accounts for.
     *
     * <p>And they explain {@code DriveConstants.kMaxSpeedMetersPerSecond}. The doubly-wrong model
     * computed a free speed of <b>4.8037 m/s</b>, and that cap is set to <b>4.8</b>. It was never a
     * chosen driveability limit — it is a fossil of the wrong model, set equal to a free speed that
     * was not the free speed. True capability is 6.0149 m/s, so the cap now leaves <b>20%</b> of the
     * drivetrain unused. Raising it is a driveability decision, so it is deliberately left alone.
     */
    public static final double kDrivingMotorFreeSpeedRps = 6784 / 60.0;
    /**
     * REV MAXSwerve 3 inch wheel, nominal diameter.
     *
     * <p>0.0762 m is exactly 3.00 in, so this is the correct nominal figure. The
     * <em>effective</em> rolling diameter is normally slightly smaller once tread
     * compression and wear are accounted for, which is what makes odometry over-report
     * distance.
     *
     * <p>{@code Calibration/WheelScale/Estimate} measures that difference against AprilTag
     * ground truth — expect a value a little under 1.0. Multiply this constant by it.
     */
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    /**
     * Total drive reduction, motor turns per wheel turn. <b>4.50:1 as built.</b>
     *
     * <p>Two stages: a 45T bevel gear on a 15T bevel pinion (3.00:1), then the 21T spur on the 14T
     * driving pinion (1.50:1). Both counts are now named constants rather than literals, so the
     * expression states which robot it describes.
     *
     * <p>Read {@code (45.0 * 22) / (14 * 15)} = 4.7143 before, which is the template's "High" ratio.
     */
    public static final double kDrivingMotorReduction =
        (45.0 * kDrivingMotorSpurTeeth) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  /**
   * Measured drive feedforward, in the form {@code V = kS*sgn(v) + kV*v + kA*a}.
   *
   * <p><b>All three are MEASURE.</b> Produced by {@code DriveSysId}, which runs WPILib SysId and
   * fits them on the robot. Paste the MEAN line from its report here.
   *
   * <p>Note what currently uses what. {@link frc.robot.common.swerve.Configs} derives its velocity
   * feedforward from free speed as {@code 12 V / kDriveWheelFreeSpeedRps}, which is a theoretical kV
   * — correct only for a motor exactly matching its datasheet, driving a wheel of exactly its nominal
   * diameter, on a fresh battery. It is a reasonable starting point and it is not a measurement.
   * Once {@link #kV} here is measured it should replace that derivation.
   *
   * <p><b>{@link #kA} has no theoretical stand-in at all</b>, which is why it is currently unused
   * rather than approximated. It converts a commanded acceleration into volts, so it is what
   * second-order kinematics needs: first-order kinematics turns a chassis velocity into module
   * velocities, second-order also turns a chassis <em>acceleration</em> into module accelerations,
   * and without kA there is nothing to turn those into an output with.
   */
  public static final class DriveFeedforwardConstants {

    /** MEASURE — volts to overcome static friction. Expect roughly 0.1 to 0.3 V. */
    public static final double kS = 0.0;

    /**
     * MEASURE — volts per metre per second.
     *
     * <p>Theory says {@code 12 / kDriveWheelFreeSpeedRps} = about 2.09 for this drivetrain. A
     * measured value materially above that means the drivetrain is losing speed somewhere theory
     * does not model: drag, tread compression, or a battery that was not fresh.
     */
    public static final double kV = 0.0;

    /**
     * MEASURE — volts per metre per second squared.
     *
     * <p>No theoretical default is offered on purpose. kA depends on the rotational inertia of the
     * whole drivetrain reflected through the gearing, which is not derivable from the constants in
     * this file, and a guessed kA in a feedforward is worse than none — it commands voltage
     * proportional to a number nobody measured.
     */
    public static final double kA = 0.0;

    /** @return true once the feedforward has actually been measured. */
    public static boolean isMeasured() {
      return kV > 0;
    }

    private DriveFeedforwardConstants() {
    }
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    /**
     * Distance between the left and right <b>module rotation axes</b>, in metres.
     *
     * <p><b>23.5 in, measured in CAD.</b> This read 26.5 in, which is the <b>frame perimeter</b>, not
     * the module spacing — the modules sit 1.5 in inboard of each rail, so 26.5 &minus; 2&times;1.5 =
     * 23.5. The two numbers differ by exactly the inset, which is why the wrong one looked right.
     *
     * <p>That was a <b>12.8% kinematics error</b>, and it is not a subtle one. The kinematics converts
     * chassis motion into module motion using this radius, so an over-large value makes the robot
     * rotate faster than commanded and mixes rotation into translation whenever both are asked for at
     * once. It reads as a tuning problem and no gain change fixes it.
     *
     * <p>Note this is not the same error as the PathPlanner disagreement that was being chased — that
     * was 6.5 mm per side between 26.5 and 27.01 in. Both of those were describing the frame. CAD says
     * neither was the module spacing.
     */
    public static final double kTrackWidth = Units.inchesToMeters(23.5);

    /** Distance between the front and rear module rotation axes. Square chassis, so identical. */
    public static final double kWheelBase = Units.inchesToMeters(23.5);
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

    // Swerve CAN IDs.
    //
    // The driving controllers are SPARK Flex, not SPARK MAX — this comment said SPARK MAX, which
    // is wrong and is worth more than a typo here: on this robot the controller type is how you
    // identify the motor (SPARK Flex means Vortex, SPARK MAX means NEO 2.0 unless it is steering).
    // A comment naming the wrong controller is how the drive free-speed bug above gets made again.
    public static final int kFrontLeftDrivingCanId = 5;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 1;

    // Steering: SPARK MAX with NEO 550, the exception to the SPARK MAX rule.
    public static final int kFrontLeftTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 2;

    public static final boolean kGyroReversed = false;
  }

  /**
   * Gains for PathPlanner's holonomic path-following controller.
   *
   * <p><strong>These have never been evaluated against correct robot behaviour.</strong>
   * Until this branch, {@code driveRobotRelative} multiplied every PathPlanner request by
   * the maximum speed before desaturation clamped it, so every path ran flat out regardless
   * of its velocity profile. Any apparent tuning was therefore compensating for that bug.
   *
   * <p>Two specific reasons to distrust the values below:
   * <ul>
   *   <li>{@code ROTATION_*} is byte-identical to the module <em>steering</em> PID in
   *       {@link frc.robot.common.swerve.Configs} — a different plant with different units.
   *       That is a copy-paste signature, not a tuned result.</li>
   *   <li>A translation P of 14.0 is aggressive for a holonomic controller, where low single
   *       digits are more usual.</li>
   * </ul>
   *
   * <p>They are preserved rather than guessed at, because changing control gains without
   * evidence is not an improvement. Re-tune both on the robot now that path velocities are
   * honoured, and use the {@code TunableNumber} wiring to do it without a redeploy.
   */
  public static class PathFollowingConstants {
    public static final double TRANSLATION_P = 14.0;
    public static final double TRANSLATION_I = 0.0;
    public static final double TRANSLATION_D = 0.1;

    public static final double ROTATION_P = 2.1;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.2;
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