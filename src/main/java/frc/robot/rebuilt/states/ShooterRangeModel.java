package frc.robot.rebuilt.states;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Continuous flywheel speed as a function of distance to the hub.
 *
 * <p>The robot already knew four discrete speeds — HUB, TOWER, TRENCH and CORNER — and it
 * already knew where each of those places is on the field. Pairing the two gives a
 * distance-to-RPM curve for free, so the shooter can be set for wherever the robot actually is
 * rather than only for the four places someone thought to press a button from.
 *
 * <p>The table below is derived from the scoring-location geometry:
 * <pre>
 *   hub face      (13.0, 4.0) to hub (11.0, 4.0)  = 2.00 m  -> 2100 RPM  (HUB preset)
 *   tower, interpolated between hub and trench     = 3.40 m  -> 2900 RPM  (TOWER preset)
 *   trench        (13.2, 7.5) to hub (11.0, 4.0)  = 4.13 m  -> 3250 RPM  (TRENCH preset)
 *   corner        (16.0, 7.5) to hub (11.0, 4.0)  = 6.10 m  -> 4500 RPM  (CORNER preset)
 * </pre>
 *
 * <p>These are the team's own tuned speeds, so the curve inherits whatever was learned by
 * shooting from those spots — it is interpolation between measured points, not a physics model.
 * Outside the calibrated range it clamps rather than extrapolating: a shot from further away
 * than anyone has ever tried should not be attempted on the strength of a straight-line guess.
 */
public final class ShooterRangeModel {

    /** Closest distance the curve is calibrated for, in metres. */
    public static final double MIN_RANGE_METERS = 2.00;

    /** Furthest distance the curve is calibrated for, in metres. */
    public static final double MAX_RANGE_METERS = 6.10;

    private static final InterpolatingDoubleTreeMap DISTANCE_TO_RPM =
            new InterpolatingDoubleTreeMap();

    static {
        DISTANCE_TO_RPM.put(2.00, 2100.0);
        DISTANCE_TO_RPM.put(3.40, 2900.0);
        DISTANCE_TO_RPM.put(4.13, 3250.0);
        DISTANCE_TO_RPM.put(6.10, 4500.0);
    }

    private ShooterRangeModel() {
    }

    /**
     * Flywheel speed for a given distance to the hub.
     *
     * @param distanceMeters Distance from robot to hub centre.
     * @return the RPM to command, clamped to the calibrated range.
     */
    public static double rpmForDistance(double distanceMeters) {
        double clamped = MathUtil.clamp(distanceMeters, MIN_RANGE_METERS, MAX_RANGE_METERS);
        return DISTANCE_TO_RPM.get(clamped);
    }

    /**
     * @param distanceMeters Distance from robot to hub centre.
     * @return true when the distance is inside the calibrated range, so the answer is
     *     interpolated rather than clamped.
     */
    public static boolean isInRange(double distanceMeters) {
        return distanceMeters >= MIN_RANGE_METERS && distanceMeters <= MAX_RANGE_METERS;
    }

    /**
     * Distance from a robot pose to the hub.
     *
     * @param robotPose Current robot pose.
     * @param hubPose   Our alliance's hub pose.
     * @return distance in metres.
     */
    public static double distanceToHub(Pose2d robotPose, Pose2d hubPose) {
        return robotPose.getTranslation().getDistance(hubPose.getTranslation());
    }

    /**
     * The heading that points the robot's shooter at the hub.
     *
     * <p>Returns the bearing from robot to hub, so the front of the robot faces the goal. If
     * the shooter fires out of the back, add 180 degrees at the call site rather than changing
     * this — the geometry here is deliberately about "which way is the hub".
     *
     * @param robotPose Current robot pose.
     * @param hubPose   Our alliance's hub pose.
     * @return the field-relative heading to face.
     */
    public static Rotation2d headingToHub(Pose2d robotPose, Pose2d hubPose) {
        Translation2d delta = hubPose.getTranslation().minus(robotPose.getTranslation());
        return new Rotation2d(delta.getX(), delta.getY());
    }
}
