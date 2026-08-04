package frc.robot.rebuilt.states;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

/**
 * Decides how the robot should behave based on where it is and what the game is doing.
 *
 * <p>Localisation is only useful if something acts on it. This turns the pose estimate and the
 * field state into two concrete assists:
 *
 * <ul>
 *   <li><b>Aim at the goal</b> — on our own half of the field with our hub open, face the hub and
 *       set the flywheel to the speed that distance needs. The driver stops thinking about which
 *       preset button to press.</li>
 *   <li><b>Cross the bump backwards</b> — approaching or on the bump, turn so the back of the
 *       robot leads. Committing before the wheels start climbing is the whole point; turning
 *       halfway up is worse than not turning.</li>
 * </ul>
 *
 * <p><b>The driver keeps translation, always.</b> These states only ever supply a heading and a
 * flywheel speed. A driver-assist that steals the sticks is a driver-assist people switch off, and
 * more importantly the driver is the one who can see a defender.
 *
 * <p>Bump crossing outranks aiming: aiming is an optimisation, whereas crossing an obstacle in the
 * wrong orientation risks getting stuck. Both are suppressed when the pose estimate is not
 * trustworthy, because acting confidently on a bad pose is worse than not acting.
 */
public class RobotStateMachine {

    /** The behaviours this robot can select, in priority order. */
    public enum State {
        /** No assist: pose untrustworthy, or nothing applies. Driver has full control. */
        MANUAL,
        /** Approaching or on the bump — reverse so the back crosses first. */
        BUMP_REVERSE,
        /** Own half, hub open — face the hub and spin up for the range. */
        AIM_AT_HUB
    }

    /** What the active state wants the robot to do. */
    public record StateOutput(
            State state,
            Optional<Rotation2d> headingTarget,
            OptionalDouble shooterRpm,
            String reason) {

        /** @return true when the state wants to control heading. */
        public boolean hasHeadingTarget() {
            return headingTarget.isPresent();
        }

        /** @return true when the state wants to set the flywheel. */
        public boolean hasShooterTarget() {
            return shooterRpm.isPresent();
        }
    }

    private static final StateOutput MANUAL_OUTPUT = new StateOutput(
            State.MANUAL, Optional.empty(), OptionalDouble.empty(), "no assist active");

    private State lastState = State.MANUAL;

    /**
     * Chooses a state and its outputs.
     *
     * @param robotPose       Current fused pose, blue-origin frame.
     * @param travelDirection Field-relative travel direction; magnitude may be anything.
     * @param hubPose         Our alliance's hub pose, or an empty pose if unknown.
     * @param fieldLength     Field length in metres.
     * @param isRedAlliance   True when on red.
     * @param hubActive       True when our hub is currently scoring.
     * @param poseTrustworthy True when the pose estimate is good enough to act on.
     * @return the selected state and what it wants.
     */
    public StateOutput update(
            Pose2d robotPose,
            Translation2d travelDirection,
            Pose2d hubPose,
            double fieldLength,
            boolean isRedAlliance,
            boolean hubActive,
            boolean poseTrustworthy) {

        StateOutput output = select(robotPose, travelDirection, hubPose, fieldLength,
                isRedAlliance, hubActive, poseTrustworthy);

        if (output.state() != lastState) {
            System.out.println("[state] " + lastState + " -> " + output.state()
                    + " (" + output.reason() + ")");
            lastState = output.state();
        }

        log(output, robotPose, hubPose);
        return output;
    }

    private StateOutput select(
            Pose2d robotPose,
            Translation2d travelDirection,
            Pose2d hubPose,
            double fieldLength,
            boolean isRedAlliance,
            boolean hubActive,
            boolean poseTrustworthy) {

        if (!poseTrustworthy) {
            return new StateOutput(State.MANUAL, Optional.empty(), OptionalDouble.empty(),
                    "pose estimate not trustworthy");
        }

        // Bump first: getting across an obstacle correctly beats optimising a shot.
        if (FieldRegions.isApproachingBump(robotPose, travelDirection)) {
            // Face opposite the direction of travel, so the back leads.
            Rotation2d travelHeading =
                    new Rotation2d(travelDirection.getX(), travelDirection.getY());
            Rotation2d reversed = travelHeading.plus(Rotation2d.fromDegrees(180));

            return new StateOutput(State.BUMP_REVERSE, Optional.of(reversed),
                    OptionalDouble.empty(),
                    FieldRegions.isOnBump(robotPose) ? "on the bump" : "approaching the bump");
        }

        boolean hubKnown = hubPose.getTranslation().getNorm() > 1e-6;
        if (hubKnown
                && hubActive
                && FieldRegions.isOnOwnSide(robotPose, fieldLength, isRedAlliance)) {

            double distance = ShooterRangeModel.distanceToHub(robotPose, hubPose);
            // The heading that aims the SHOOTER, not the chassis nose. The shooter sits 90 degrees
            // off forward on this robot, so those are not the same thing and using the bearing
            // directly would fire the shot sideways off the field.
            Rotation2d heading = ShooterRangeModel.headingToAimShooter(robotPose, hubPose);
            double rpm = ShooterRangeModel.rpmForDistance(distance);

            String reason = ShooterRangeModel.isInRange(distance)
                    ? String.format("own side, hub open, %.2f m", distance)
                    : String.format("own side, hub open, %.2f m (outside calibrated range, "
                            + "speed clamped)", distance);

            return new StateOutput(State.AIM_AT_HUB, Optional.of(heading),
                    OptionalDouble.of(rpm), reason);
        }

        return MANUAL_OUTPUT;
    }

    private void log(StateOutput output, Pose2d robotPose, Pose2d hubPose) {
        Logger.recordOutput("States/Active", output.state().name());
        Logger.recordOutput("States/Reason", output.reason());
        Logger.recordOutput("States/HasHeadingTarget", output.hasHeadingTarget());
        Logger.recordOutput("States/HeadingTargetDeg",
                output.headingTarget().map(Rotation2d::getDegrees).orElse(Double.NaN));
        Logger.recordOutput("States/ShooterRpmTarget",
                output.shooterRpm().orElse(Double.NaN));

        boolean hubKnown = hubPose.getTranslation().getNorm() > 1e-6;
        Logger.recordOutput("States/DistanceToHub", hubKnown
                ? ShooterRangeModel.distanceToHub(robotPose, hubPose)
                : Double.NaN);
        Logger.recordOutput("States/OnBump", FieldRegions.isOnBump(robotPose));
    }

    /** @return the state selected on the previous update. */
    public State getLastState() {
        return lastState;
    }

    /** Resets the remembered state, so the next transition is announced. */
    public void reset() {
        lastState = State.MANUAL;
    }
}
