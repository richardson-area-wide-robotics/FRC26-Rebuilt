package frc.robot.rebuilt.states;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Where on the field the robot is, in terms the robot's behaviour cares about.
 *
 * <p>Regions are expressed in the blue-origin field frame WPILib uses, and mirrored for the red
 * alliance rather than duplicated, so there is one definition per region and no chance of the
 * two copies drifting apart.
 *
 * <p><strong>The bump band is a placeholder and must be measured.</strong> Everything else here
 * derives from the field length, which is read from the AprilTag layout, but the bump's position
 * is a physical fact about the field that no library knows.
 */
public final class FieldRegions {

    /**
     * MEASURE — the along-field extent of the bump, in metres from the blue wall.
     *
     * <p>The values below describe a band across the middle of the field, which is where a bump
     * usually sits, widened slightly so the robot commits to its approach before it arrives
     * rather than while climbing. Replace both numbers with a tape measure from the real field:
     * if the band is wrong the robot will either reverse for no reason or fail to reverse when
     * it matters.
     */
    public static final double BUMP_NEAR_EDGE_METERS = 7.2;

    /** MEASURE — see {@link #BUMP_NEAR_EDGE_METERS}. */
    public static final double BUMP_FAR_EDGE_METERS = 9.4;

    /**
     * How far before the bump to start turning, in metres.
     *
     * <p>Rotating 180 degrees takes time, and doing it while a wheel is already climbing is how
     * a robot ends up crossing sideways. Committing early is the point.
     */
    public static final double BUMP_APPROACH_MARGIN_METERS = 1.0;

    private FieldRegions() {
    }

    /**
     * Whether a pose is on our own half of the field.
     *
     * @param pose            Robot pose in the blue-origin frame.
     * @param fieldLength     Field length in metres, from the AprilTag layout.
     * @param isRedAlliance   True when on red.
     * @return true when the robot is on its own half.
     */
    public static boolean isOnOwnSide(Pose2d pose, double fieldLength, boolean isRedAlliance) {
        double midfield = fieldLength / 2.0;
        return isRedAlliance ? pose.getX() > midfield : pose.getX() < midfield;
    }

    /**
     * Whether a pose is inside the bump band.
     *
     * @param pose Robot pose in the blue-origin frame.
     * @return true when on the bump.
     */
    public static boolean isOnBump(Pose2d pose) {
        return pose.getX() >= BUMP_NEAR_EDGE_METERS && pose.getX() <= BUMP_FAR_EDGE_METERS;
    }

    /**
     * Whether a pose is close enough to the bump, and heading towards it, to start turning.
     *
     * <p>Direction matters: a robot sitting just past the bump driving away from it should not
     * spin around. Approach is judged from the commanded travel direction rather than from
     * heading, because a swerve chassis can drive one way while facing another.
     *
     * @param pose             Robot pose in the blue-origin frame.
     * @param travelDirection  Field-relative direction of travel, as a unit-ish vector. A near
     *                         zero vector means the robot is not really moving.
     * @return true when the bump is being approached and is close.
     */
    public static boolean isApproachingBump(Pose2d pose, Translation2d travelDirection) {
        if (travelDirection.getNorm() < 1e-3) {
            return false; // Stationary: nothing to commit to.
        }

        double x = pose.getX();
        boolean movingTowardsFarSide = travelDirection.getX() > 0;

        if (isOnBump(pose)) {
            return true;
        }

        if (movingTowardsFarSide) {
            // Approaching the near edge from the blue side.
            return x < BUMP_NEAR_EDGE_METERS
                    && x >= BUMP_NEAR_EDGE_METERS - BUMP_APPROACH_MARGIN_METERS;
        }

        // Approaching the far edge from the red side.
        return x > BUMP_FAR_EDGE_METERS
                && x <= BUMP_FAR_EDGE_METERS + BUMP_APPROACH_MARGIN_METERS;
    }

    /** @return the width of the bump band, in metres. */
    public static double bumpWidthMeters() {
        return BUMP_FAR_EDGE_METERS - BUMP_NEAR_EDGE_METERS;
    }
}
