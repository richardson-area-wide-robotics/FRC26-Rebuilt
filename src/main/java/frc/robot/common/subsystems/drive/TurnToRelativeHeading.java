package frc.robot.common.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/**
 * Turns in place by a relative angle, tracking total rotation rather than absolute heading.
 *
 * <p>Accumulating the rotation matters: a 270 degree turn cannot be commanded as an absolute
 * heading target, because the controller would take the 90 degree short way round instead. So
 * this integrates the shortest per-loop delta and stops once the total reaches the request.
 *
 * <p>Uses the gyro rather than the fused pose. Over a turn lasting a couple of seconds gyro
 * drift is around 0.02 degrees, far below what the tags can resolve, and the gyro updates far
 * faster — so it gives a smoother turn. AprilTags still correct the absolute heading through
 * the pose estimator afterwards, and the auto-calibrator separately measures the gyro's scale
 * error against the tags.
 */
public class TurnToRelativeHeading extends Command {

    /** Tolerance for declaring the turn complete, in degrees. */
    private static final double TOLERANCE_DEGREES = 1.0;

    /** Loops inside tolerance before finishing. */
    private static final int SETTLE_LOOPS = 10;

    /** Maximum rotation command as a fraction of the configured maximum angular rate. */
    private static final double MAX_FRACTION = 0.35;

    /** Minimum command so the final degree is not approached indefinitely slowly. */
    private static final double MIN_FRACTION = 0.05;

    private final SwerveDriveSubsystem drive;
    private final double requestedDegrees;

    /** Gain in fraction-of-max-rate per degree of remaining rotation. */
    private final PIDController controller = new PIDController(0.012, 0, 0);

    private double accumulatedDegrees;
    private double lastHeading;
    private int loopsInTolerance;

    /**
     * @param drive            Drivetrain to command.
     * @param requestedDegrees Rotation to perform. Positive is counter-clockwise.
     */
    public TurnToRelativeHeading(SwerveDriveSubsystem drive, double requestedDegrees) {
        this.drive = drive;
        this.requestedDegrees = requestedDegrees;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        accumulatedDegrees = 0;
        lastHeading = drive.getHeading();
        loopsInTolerance = 0;
        controller.reset();
        Logger.recordOutput("TurnRelative/Requested", requestedDegrees);
    }

    @Override
    public void execute() {
        double heading = drive.getHeading();
        accumulatedDegrees += shortestDelta(lastHeading, heading);
        lastHeading = heading;

        double remaining = requestedDegrees - accumulatedDegrees;

        double command = MathUtil.clamp(
                controller.calculate(accumulatedDegrees, requestedDegrees),
                -MAX_FRACTION, MAX_FRACTION);

        // Keep enough authority to overcome static friction near the target.
        if (Math.abs(remaining) > TOLERANCE_DEGREES && Math.abs(command) < MIN_FRACTION) {
            command = Math.copySign(MIN_FRACTION, remaining);
        }
        if (Math.abs(remaining) <= TOLERANCE_DEGREES) {
            command = 0;
            loopsInTolerance++;
        } else {
            loopsInTolerance = 0;
        }

        drive.drive(0, 0, command, false);

        Logger.recordOutput("TurnRelative/Accumulated", accumulatedDegrees);
        Logger.recordOutput("TurnRelative/Remaining", remaining);
    }

    @Override
    public boolean isFinished() {
        return loopsInTolerance >= SETTLE_LOOPS;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, false);
        Logger.recordOutput("TurnRelative/FinalErrorDeg", requestedDegrees - accumulatedDegrees);
        System.out.printf("TurnRelative: requested %+.1f deg, achieved %+.1f deg (err %+.2f)%n",
                requestedDegrees, accumulatedDegrees, requestedDegrees - accumulatedDegrees);
    }

    /** @return total rotation accumulated so far, in degrees. */
    public double getAccumulatedDegrees() {
        return accumulatedDegrees;
    }

    /** Shortest signed angular difference, in degrees. */
    private static double shortestDelta(double fromDegrees, double toDegrees) {
        double delta = (toDegrees - fromDegrees) % 360.0;
        if (delta > 180) {
            delta -= 360;
        } else if (delta < -180) {
            delta += 360;
        }
        return delta;
    }
}
