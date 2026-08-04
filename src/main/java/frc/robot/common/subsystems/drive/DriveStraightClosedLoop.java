package frc.robot.common.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/**
 * Drives a straight line of a given length, correcting continuously from the fused pose
 * estimate rather than dead-reckoning to a distance.
 *
 * <p><b>Why this beats open-loop odometry.</b> Driving a fixed distance from wheel encoders
 * makes the endpoint error proportional to the wheel-scale error — a 1% diameter error is
 * 1.2 inches over 10 feet, before anything else contributes. Terminating on the
 * <em>AprilTag-corrected</em> pose instead removes wheel scale from the endpoint entirely: it
 * then only shapes the velocity profile, not where the robot stops.
 *
 * <p>Three loops run together, each on the sensor best suited to it:
 * <ul>
 *   <li><b>Heading</b> — held with the gyro. Short-term gyro accuracy is excellent and its
 *       drift over a few seconds is negligible (0.5°/min is 0.02° in a 2 s run), so this keeps
 *       the chassis square without waiting on camera frames.</li>
 *   <li><b>Cross-track</b> — corrected from the fused pose, which is where AprilTags enter.
 *       Actively steering back onto the line bounds lateral error to the estimator's accuracy
 *       instead of letting a steering misalignment integrate into a growing offset.</li>
 *   <li><b>Along-track</b> — distance remaining measured from the fused pose, so the stopping
 *       point is absolute rather than integrated.</li>
 * </ul>
 *
 * <p><b>What this cannot fix.</b> Accuracy is now limited by how right the pose estimate is,
 * not by the wheels. A wrong camera transform or the wrong AprilTag field layout puts the
 * robot precisely on a wrong target — and the welded/AndyMark layouts differ by 3.6 cm, which
 * is larger than a 1 inch budget. Closing the loop trades odometry error for vision
 * calibration error; it does not eliminate error.
 */
public class DriveStraightClosedLoop extends Command {

    /** Tolerance for declaring the run finished, in metres. */
    private static final double POSITION_TOLERANCE_METERS = 0.01;

    /** How many consecutive loops the robot must sit inside tolerance before finishing. */
    private static final int SETTLE_LOOPS = 10;

    /** Cruise speed as a fraction of maximum. Slow enough that slip stays small. */
    private static final double CRUISE_FRACTION = 0.25;

    /** Speed floor so the last few centimetres are not approached infinitely slowly. */
    private static final double MIN_FRACTION = 0.04;

    private final SwerveDriveSubsystem drive;
    private final double distanceMeters;

    /**
     * Cross-track correction. Gain is in fraction-of-max-speed per metre of lateral error, so
     * 2.0 means a 10 cm offset commands 20% of maximum sideways speed.
     */
    private final PIDController crossTrackController = new PIDController(2.0, 0, 0);

    /** Heading hold, in fraction-of-max-angular-rate per degree. */
    private final PIDController headingController = new PIDController(0.02, 0, 0);

    private Pose2d startPose;
    private Rotation2d targetHeading;
    private Translation2d unitDirection;
    private int loopsInTolerance;

    /**
     * @param drive          Drivetrain to command.
     * @param distanceMeters Distance to travel along the current heading. Negative reverses.
     */
    public DriveStraightClosedLoop(SwerveDriveSubsystem drive, double distanceMeters) {
        this.drive = drive;
        this.distanceMeters = distanceMeters;
        headingController.enableContinuousInput(-180, 180);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        startPose = drive.getPose();
        targetHeading = startPose.getRotation();
        // Travel along whatever direction the robot is already facing.
        unitDirection = new Translation2d(targetHeading.getCos(), targetHeading.getSin());
        loopsInTolerance = 0;

        crossTrackController.reset();
        headingController.reset();

        Logger.recordOutput("DriveStraight/StartPose", startPose);
        Logger.recordOutput("DriveStraight/TargetDistance", distanceMeters);
    }

    @Override
    public void execute() {
        Pose2d pose = drive.getPose();
        Translation2d travelled = pose.getTranslation().minus(startPose.getTranslation());

        // Decompose measured displacement along and across the commanded line.
        double along = travelled.getX() * unitDirection.getX()
                + travelled.getY() * unitDirection.getY();
        double cross = -travelled.getX() * unitDirection.getY()
                + travelled.getY() * unitDirection.getX();

        double remaining = distanceMeters - along;

        // Along-track: cruise, then taper inside the last 40 cm so the stop is not abrupt.
        double alongCommand = Math.copySign(
                Math.min(CRUISE_FRACTION, Math.max(MIN_FRACTION, Math.abs(remaining) * 0.6)),
                remaining);
        if (Math.abs(remaining) <= POSITION_TOLERANCE_METERS) {
            alongCommand = 0;
        }

        // Cross-track: steer back onto the line. Negated because the controller drives the
        // error to zero and cross is the error itself.
        double crossCommand = MathUtil.clamp(
                -crossTrackController.calculate(cross, 0), -CRUISE_FRACTION, CRUISE_FRACTION);

        // Heading: hold the entry heading using the gyro.
        double headingCommand = MathUtil.clamp(
                headingController.calculate(drive.getHeading(), targetHeading.getDegrees()),
                -0.3, 0.3);

        // Convert along/cross into field-relative x/y.
        double xCommand = alongCommand * unitDirection.getX() - crossCommand * unitDirection.getY();
        double yCommand = alongCommand * unitDirection.getY() + crossCommand * unitDirection.getX();

        drive.drive(xCommand, yCommand, headingCommand, true);

        if (Math.abs(remaining) <= POSITION_TOLERANCE_METERS) {
            loopsInTolerance++;
        } else {
            loopsInTolerance = 0;
        }

        Logger.recordOutput("DriveStraight/AlongMeters", along);
        Logger.recordOutput("DriveStraight/CrossMeters", cross);
        Logger.recordOutput("DriveStraight/RemainingMeters", remaining);
        Logger.recordOutput("DriveStraight/HeadingErrorDeg",
                MathUtil.inputModulus(targetHeading.getDegrees() - drive.getHeading(), -180, 180));
    }

    @Override
    public boolean isFinished() {
        return loopsInTolerance >= SETTLE_LOOPS;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, false);

        Pose2d pose = drive.getPose();
        Translation2d travelled = pose.getTranslation().minus(startPose.getTranslation());
        double along = travelled.getX() * unitDirection.getX()
                + travelled.getY() * unitDirection.getY();
        double cross = -travelled.getX() * unitDirection.getY()
                + travelled.getY() * unitDirection.getX();

        Logger.recordOutput("DriveStraight/FinalAlongError", along - distanceMeters);
        Logger.recordOutput("DriveStraight/FinalCrossError", cross);
        Logger.recordOutput("DriveStraight/Interrupted", interrupted);

        System.out.printf(
                "DriveStraight: commanded %.3f m, fused along %.3f m (err %.1f mm), "
                        + "cross %.1f mm%n",
                distanceMeters, along, (along - distanceMeters) * 1000.0, cross * 1000.0);
    }

    /** @return the measured along-track distance since the command started, in metres. */
    public double getAlongTrackMeters() {
        Pose2d pose = drive.getPose();
        Translation2d travelled = pose.getTranslation().minus(startPose.getTranslation());
        return travelled.getX() * unitDirection.getX() + travelled.getY() * unitDirection.getY();
    }
}
