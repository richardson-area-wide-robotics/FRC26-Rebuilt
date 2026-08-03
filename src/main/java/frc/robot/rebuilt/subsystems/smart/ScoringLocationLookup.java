package frc.robot.rebuilt.subsystems.smart;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.AllArgsConstructor;

/**
 * Look-up of the nearest useful field position for the current alliance.
 *
 * <p>Every location is stored twice, once per alliance, and {@link #findClosest} returns
 * whichever mirrored pose is nearest the robot. Until an alliance is known the lookup
 * returns an empty pose, so callers get a defined value rather than a wrong one.
 *
 * <p>Note that nothing acts on the result yet — the container logs it for the driver. Wiring
 * it into shooter preset selection is the obvious next step.
 */
public final class ScoringLocationLookup {

    private static final int LOCATION_COUNT = 8;

    private static final ScoringLocation[] scoringLocations = new ScoringLocation[LOCATION_COUNT];

    /** True for red, false for blue, null until the driver station tells us. */
    private static Boolean redAlliance;

    private static final Pose2d hubRedPose = new Pose2d(11, 4, new Rotation2d());
    private static final Pose2d hubBluePose = new Pose2d(4.5, 4, new Rotation2d());

    private ScoringLocationLookup() {
    }

    /**
     * Sets which alliance's mirrored poses to return.
     *
     * <p>Replaces a public mutable {@code Boolean team} field that callers assigned directly.
     *
     * @param isRed true when on the red alliance.
     */
    public static void setRedAlliance(boolean isRed) {
        redAlliance = isRed;
    }

    /** Clears the alliance, returning the lookup to its unknown state. Used by tests. */
    public static void clearAlliance() {
        redAlliance = null;
    }

    /** @return true once an alliance has been set. */
    public static boolean hasAlliance() {
        return redAlliance != null;
    }

    public static void buildScoringLocations() {
        scoringLocations[0] = new ScoringLocation("right_corner", new Pose2d(16, 7.5, new Rotation2d()), new Pose2d(0.5, 7.5, new Rotation2d()));
        scoringLocations[1] = new ScoringLocation("left_corner", new Pose2d(16, 0.5, new Rotation2d()), new Pose2d(0.5, 0.5, new Rotation2d()));
        scoringLocations[2] = new ScoringLocation("left_trench", new Pose2d(13.2, 0.5, new Rotation2d()), new Pose2d(3.5, 7.5, new Rotation2d()));
        scoringLocations[3] = new ScoringLocation("right_trench", new Pose2d(13.2, 7.5, new Rotation2d()), new Pose2d(3.5, 0.5, new Rotation2d()));
        scoringLocations[4] = new ScoringLocation("climber", new Pose2d(14.5, 4, new Rotation2d()), new Pose2d(2, 3.5, new Rotation2d()));
        scoringLocations[5] = new ScoringLocation("rightside_hub", new Pose2d(13, 5, new Rotation2d()), new Pose2d(3.5, 5, new Rotation2d()));
        scoringLocations[6] = new ScoringLocation("hub", new Pose2d(13, 4, new Rotation2d()), new Pose2d(3.5, 4, new Rotation2d()));
        scoringLocations[7] = new ScoringLocation("leftside_hub", new Pose2d(13, 3, new Rotation2d()), new Pose2d(3.5, 3, new Rotation2d()));
    }

    /** @return true once {@link #buildScoringLocations()} has populated the table. */
    public static boolean isBuilt() {
        for (ScoringLocation location : scoringLocations) {
            if (location == null) {
                return false;
            }
        }
        return true;
    }

    /**
     * Finds the alliance-correct scoring location nearest a pose.
     *
     * @param robotPose Current robot pose.
     * @return The nearest location's pose, or an empty pose when the alliance is unknown or
     *         the table has not been built.
     */
    public static Pose2d findClosest(Pose2d robotPose) {
        if (redAlliance == null || robotPose == null || !isBuilt()) {
            return new Pose2d();
        }

        double closestDist = Double.MAX_VALUE;
        Pose2d closestPose = new Pose2d();

        for (ScoringLocation location : scoringLocations) {
            Pose2d candidate = redAlliance ? location.redPose2d : location.bluePose2d;
            double distance = robotPose.getTranslation().getDistance(candidate.getTranslation());
            if (distance < closestDist) {
                closestDist = distance;
                closestPose = candidate;
            }
        }

        return closestPose;
    }

    /**
     * @return our alliance's hub pose, or an empty pose when the alliance is unknown.
     */
    public static Pose2d findHub() {
        if (redAlliance == null) {
            return new Pose2d();
        }
        return redAlliance ? hubRedPose : hubBluePose;
    }

    /**
     * Finds the nearest location by name, for tests and diagnostics.
     *
     * @param robotPose Current robot pose.
     * @return The nearest location's name, or an empty string when unavailable.
     */
    public static String findClosestName(Pose2d robotPose) {
        if (redAlliance == null || robotPose == null || !isBuilt()) {
            return "";
        }

        double closestDist = Double.MAX_VALUE;
        String closestName = "";

        for (ScoringLocation location : scoringLocations) {
            Pose2d candidate = redAlliance ? location.redPose2d : location.bluePose2d;
            double distance = robotPose.getTranslation().getDistance(candidate.getTranslation());
            if (distance < closestDist) {
                closestDist = distance;
                closestName = location.name;
            }
        }

        return closestName;
    }

    @AllArgsConstructor
    static class ScoringLocation {
        public String name;
        public Pose2d redPose2d;
        public Pose2d bluePose2d;
    }
}
