package frc.robot.common.components.diagnostics;

/**
 * Tracks total rotation across successive wrapped heading readings.
 *
 * <p>Gyros and pose estimates report heading wrapped to ±180°, which loses the difference between
 * turning 10° and turning 370°. Anything that needs total rotation — a commanded 270° turn, or
 * several full revolutions during a gyro-scale sweep — has to unwrap the readings and accumulate
 * the per-sample deltas instead.
 *
 * <p>This is why {@code TurnToRelativeHeading} cannot simply servo to an absolute heading target: a
 * controller asked to reach "current heading plus 270°" would compute the shortest path and turn
 * 90° the other way, arriving at the right heading having done the wrong thing. Every manoeuvre in
 * the calibration catalogue that contains a 270° turn depends on this distinction.
 *
 * <p>Extracted because the same unwrapping logic was written twice, in
 * {@code TurnToRelativeHeading} and {@code DriveAutoCalibrator}, and used in five places. Pure
 * arithmetic, so it is directly testable.
 */
public class RotationAccumulator {

    private double accumulatedDegrees;
    private double lastHeadingDegrees;
    private boolean started;

    /**
     * Begins accumulating from a starting heading.
     *
     * <p>Safe to call again to restart; the running total resets to zero.
     *
     * @param headingDegrees Current heading, wrapped or otherwise.
     */
    public void reset(double headingDegrees) {
        accumulatedDegrees = 0;
        lastHeadingDegrees = headingDegrees;
        started = true;
    }

    /**
     * Folds in a new heading reading.
     *
     * <p>The first call after construction establishes the reference without accumulating, so a
     * caller that forgets to {@link #reset} does not record a spurious jump from zero.
     *
     * @param headingDegrees Latest heading reading.
     * @return the running total rotation, in degrees.
     */
    public double update(double headingDegrees) {
        if (!started) {
            reset(headingDegrees);
            return 0;
        }

        accumulatedDegrees += shortestDelta(lastHeadingDegrees, headingDegrees);
        lastHeadingDegrees = headingDegrees;
        return accumulatedDegrees;
    }

    /** @return total rotation since the last reset, in degrees. Positive is counter-clockwise. */
    public double getAccumulatedDegrees() {
        return accumulatedDegrees;
    }

    /** @return the most recent heading reading. */
    public double getLastHeadingDegrees() {
        return lastHeadingDegrees;
    }

    /**
     * Shortest signed angular difference between two headings, in degrees.
     *
     * <p>Always in (−180, 180], which is what makes accumulation correct: between consecutive
     * samples the robot cannot have turned more than half a revolution, so the shortest
     * interpretation is the right one. That assumption fails only if sampling is slower than the
     * robot can spin 180°, which at a 20 ms loop would need well over 1500 °/s.
     *
     * @param fromDegrees Earlier heading.
     * @param toDegrees   Later heading.
     * @return signed difference, positive counter-clockwise.
     */
    public static double shortestDelta(double fromDegrees, double toDegrees) {
        double delta = (toDegrees - fromDegrees) % 360.0;
        if (delta > 180) {
            delta -= 360;
        } else if (delta < -180) {
            delta += 360;
        }
        return delta;
    }
}
