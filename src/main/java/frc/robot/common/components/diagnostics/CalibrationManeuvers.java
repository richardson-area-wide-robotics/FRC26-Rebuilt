package frc.robot.common.components.diagnostics;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The catalogue of calibration manoeuvres, and the geometry that says where each one should
 * finish.
 *
 * <p>A manoeuvre is a list of legs — drive a signed distance, or turn a signed angle. Because
 * the expected finishing pose can be computed analytically from the legs, every run has a
 * ground truth to compare the measured pose against, and that comparison is what the
 * calibration reports.
 *
 * <p>Three families of manoeuvre, each answering a different question:
 * <ul>
 *   <li><b>Permutations</b> — drive, turn, drive. Sweeping distance, turn size and direction
 *       exposes errors that only appear in one combination, such as a turn that is accurate
 *       clockwise but not counter-clockwise.</li>
 *   <li><b>Out and back, same path</b> — the closure error on returning to the start. Note that
 *       a pure wheel-scale error largely <em>cancels</em> here, because both legs are wrong by
 *       the same proportion in opposite directions. What survives is heading error, backlash
 *       and hysteresis, which makes this the sharpest test for those specifically.</li>
 *   <li><b>Out and back, different path</b> — return by a different route, so the legs no longer
 *       cancel. This is where scale error and heading error compound instead of subtracting,
 *       and it is the closest analogue to a real autonomous path.</li>
 * </ul>
 *
 * <p>Pure geometry, no hardware, so the expected poses are unit tested.
 */
public final class CalibrationManeuvers {

    /** Ten feet, the distance the accuracy requirement is written against. */
    public static final double TEN_FEET_METERS = Units.feetToMeters(10);

    /** Five feet. */
    public static final double FIVE_FEET_METERS = Units.feetToMeters(5);

    private CalibrationManeuvers() {
    }

    /** What a leg does. */
    public enum LegType {
        /** Drive along the current heading. Value is signed metres; negative reverses. */
        DRIVE,
        /** Turn in place. Value is signed degrees; positive is counter-clockwise (left). */
        TURN
    }

    /**
     * One step of a manoeuvre.
     *
     * @param type  Whether this leg drives or turns.
     * @param value Signed metres for a drive, signed degrees for a turn.
     */
    public record Leg(LegType type, double value) {

        /** @param meters signed distance; negative reverses. @return a driving leg. */
        public static Leg drive(double meters) {
            return new Leg(LegType.DRIVE, meters);
        }

        /** @param degrees signed angle, positive left. @return a turning leg. */
        public static Leg turn(double degrees) {
            return new Leg(LegType.TURN, degrees);
        }

        @Override
        public String toString() {
            return type == LegType.DRIVE
                    ? String.format("drive %+.2f m", value)
                    : String.format("turn %+.0f deg", value);
        }
    }

    /**
     * A named sequence of legs.
     *
     * @param name         Short identifier, used as the log key.
     * @param description  What the manoeuvre is testing.
     * @param legs         The steps, in order.
     * @param returnsToStart Whether the robot should finish where it began, so closure error
     *                       is the meaningful metric.
     */
    public record Maneuver(String name, String description, List<Leg> legs,
                           boolean returnsToStart) {

        /** @return total driven distance, ignoring direction, in metres. */
        public double totalDistanceMeters() {
            double total = 0;
            for (Leg leg : legs) {
                if (leg.type() == LegType.DRIVE) {
                    total += Math.abs(leg.value());
                }
            }
            return total;
        }
    }

    /**
     * Computes where a manoeuvre should finish, starting from a given pose.
     *
     * <p>Composed as rigid transforms, so it is exact: a drive leg translates along the
     * <em>current</em> heading, and a turn leg rotates in place.
     *
     * @param start Starting pose.
     * @param legs  Legs to apply.
     * @return the ideal finishing pose.
     */
    public static Pose2d expectedPose(Pose2d start, List<Leg> legs) {
        Pose2d pose = start;
        for (Leg leg : legs) {
            if (leg.type() == LegType.DRIVE) {
                pose = pose.plus(new Transform2d(
                        new Translation2d(leg.value(), 0), new Rotation2d()));
            } else {
                pose = pose.plus(new Transform2d(
                        new Translation2d(), Rotation2d.fromDegrees(leg.value())));
            }
        }
        return pose;
    }

    /**
     * Reverses a manoeuvre so the robot retraces its outbound path exactly.
     *
     * <p>Driving legs are reversed in sign so the robot backs along the same line rather than
     * turning around, and the leg order is reversed. Turns are negated too, so the robot ends
     * facing its original direction.
     *
     * @param legs Outbound legs.
     * @return legs that return along the same path.
     */
    public static List<Leg> retraceLegs(List<Leg> legs) {
        List<Leg> back = new ArrayList<>();
        for (int i = legs.size() - 1; i >= 0; i--) {
            Leg leg = legs.get(i);
            back.add(new Leg(leg.type(), -leg.value()));
        }
        return back;
    }

    // -----------------------------------------------------------------------------------
    // Family 1 — permutations of drive, turn, drive
    // -----------------------------------------------------------------------------------

    /**
     * Every combination of outbound distance, turn, and final leg direction.
     *
     * <p>{10 ft, 5 ft} outbound, times {90, 270} degrees, times {left, right}, times
     * {forward, reverse} on the final 10 ft leg — sixteen manoeuvres.
     *
     * <p>Sweeping rather than spot-checking is the point: a turn that is accurate one way and
     * not the other, or a reverse leg with more backlash than forward, only shows up when both
     * variants are run and compared.
     *
     * @return the sixteen permutations.
     */
    public static List<Maneuver> permutations() {
        List<Maneuver> maneuvers = new ArrayList<>();

        double[] outboundDistances = {TEN_FEET_METERS, FIVE_FEET_METERS};
        String[] outboundNames = {"10ft", "5ft"};

        double[] turnMagnitudes = {90, 270};
        // Positive is counter-clockwise, which is left.
        int[] turnSigns = {1, -1};
        String[] turnDirections = {"L", "R"};

        double[] finalDistances = {TEN_FEET_METERS, -TEN_FEET_METERS};
        String[] finalNames = {"fwd10ft", "rev10ft"};

        for (int d = 0; d < outboundDistances.length; d++) {
            for (double magnitude : turnMagnitudes) {
                for (int s = 0; s < turnSigns.length; s++) {
                    for (int f = 0; f < finalDistances.length; f++) {
                        String name = String.format("P_%s_%s%.0f_%s",
                                outboundNames[d], turnDirections[s], magnitude, finalNames[f]);

                        List<Leg> legs = List.of(
                                Leg.drive(outboundDistances[d]),
                                Leg.turn(magnitude * turnSigns[s]),
                                Leg.drive(finalDistances[f]));

                        String description = String.format(
                                "Drive %s, turn %.0f deg %s, then %s",
                                outboundNames[d], magnitude,
                                turnSigns[s] > 0 ? "left" : "right",
                                finalNames[f]);

                        maneuvers.add(new Maneuver(name, description, legs, false));
                    }
                }
            }
        }

        return maneuvers;
    }

    // -----------------------------------------------------------------------------------
    // Family 2 — out and back along the same path
    // -----------------------------------------------------------------------------------

    /**
     * Manoeuvres that drive out and retrace the identical path home.
     *
     * <p>Closure error — how far from the start the robot actually finishes — is the metric.
     * Be aware of what this does and does not catch: a wheel-scale error scales both legs
     * equally and in opposite directions, so it largely cancels and closure error stays small
     * even with badly calibrated wheels. Heading error, backlash and hysteresis do <em>not</em>
     * cancel, so this isolates them.
     *
     * @return the same-path out-and-back manoeuvres.
     */
    public static List<Maneuver> outAndBackSamePath() {
        List<Maneuver> maneuvers = new ArrayList<>();

        // Straight out, straight back in reverse — no turning at all, so any closure error is
        // purely drive-side hysteresis or heading drift.
        maneuvers.add(new Maneuver(
                "OB_same_straight10ft",
                "Out 10 ft forward, back 10 ft in reverse along the same line",
                List.of(Leg.drive(TEN_FEET_METERS), Leg.drive(-TEN_FEET_METERS)),
                true));

        maneuvers.add(new Maneuver(
                "OB_same_straight5ft",
                "Out 5 ft forward, back 5 ft in reverse along the same line",
                List.of(Leg.drive(FIVE_FEET_METERS), Leg.drive(-FIVE_FEET_METERS)),
                true));

        // Out, turn around, drive back forwards. Same physical path, but both legs are driven
        // forwards, so this separates reverse-direction effects from the path itself.
        maneuvers.add(new Maneuver(
                "OB_same_turnaround10ft",
                "Out 10 ft, turn 180 deg, drive 10 ft forward, turn 180 deg back",
                List.of(Leg.drive(TEN_FEET_METERS), Leg.turn(180),
                        Leg.drive(TEN_FEET_METERS), Leg.turn(180)),
                true));

        // An L out and the same L back, retraced exactly.
        List<Leg> lShape = List.of(
                Leg.drive(TEN_FEET_METERS), Leg.turn(90), Leg.drive(FIVE_FEET_METERS));
        List<Leg> lThereAndBack = new ArrayList<>(lShape);
        lThereAndBack.addAll(retraceLegs(lShape));
        maneuvers.add(new Maneuver(
                "OB_same_L_retrace",
                "L-shape out, then the identical L retraced back to the start",
                lThereAndBack,
                true));

        return maneuvers;
    }

    // -----------------------------------------------------------------------------------
    // Family 3 — out and back by a different route
    // -----------------------------------------------------------------------------------

    /**
     * Manoeuvres that return to the start by a different route.
     *
     * <p>Because the outbound and return legs are no longer mirror images, scale and heading
     * errors compound rather than cancelling. This is the harder and more realistic test, and
     * the closest analogue to an autonomous path that has to arrive somewhere specific after a
     * sequence of turns.
     *
     * @return the different-path out-and-back manoeuvres.
     */
    public static List<Maneuver> outAndBackDifferentPath() {
        List<Maneuver> maneuvers = new ArrayList<>();

        // A square. Four equal legs and four left turns should close perfectly; whatever it
        // misses by is the accumulated per-turn and per-leg error.
        maneuvers.add(new Maneuver(
                "OB_diff_square5ft",
                "5 ft square, four left turns — closure error is accumulated turn error",
                List.of(
                        Leg.drive(FIVE_FEET_METERS), Leg.turn(90),
                        Leg.drive(FIVE_FEET_METERS), Leg.turn(90),
                        Leg.drive(FIVE_FEET_METERS), Leg.turn(90),
                        Leg.drive(FIVE_FEET_METERS), Leg.turn(90)),
                true));

        // The same square the other way round, to expose direction-dependent turn error.
        maneuvers.add(new Maneuver(
                "OB_diff_square5ft_right",
                "5 ft square clockwise, to compare against the counter-clockwise square",
                List.of(
                        Leg.drive(FIVE_FEET_METERS), Leg.turn(-90),
                        Leg.drive(FIVE_FEET_METERS), Leg.turn(-90),
                        Leg.drive(FIVE_FEET_METERS), Leg.turn(-90),
                        Leg.drive(FIVE_FEET_METERS), Leg.turn(-90)),
                true));

        // Out along one side of a rectangle, back along the other — a different route home.
        maneuvers.add(new Maneuver(
                "OB_diff_rectangle",
                "10 ft out, 5 ft across, 10 ft back, 5 ft across — rectangle loop",
                List.of(
                        Leg.drive(TEN_FEET_METERS), Leg.turn(90),
                        Leg.drive(FIVE_FEET_METERS), Leg.turn(90),
                        Leg.drive(TEN_FEET_METERS), Leg.turn(90),
                        Leg.drive(FIVE_FEET_METERS), Leg.turn(90)),
                true));

        // Out with a right turn, home with lefts: mixes turn directions so a bias in one
        // direction cannot hide by being applied symmetrically.
        maneuvers.add(new Maneuver(
                "OB_diff_mixed_turns",
                "Out 10 ft, right 90, 5 ft, left 90, 10 ft back, left 90, 5 ft, left 90",
                List.of(
                        Leg.drive(TEN_FEET_METERS), Leg.turn(-90),
                        Leg.drive(FIVE_FEET_METERS), Leg.turn(-90),
                        Leg.drive(TEN_FEET_METERS), Leg.turn(-90),
                        Leg.drive(FIVE_FEET_METERS), Leg.turn(-90)),
                true));

        // A triangle, so the turns are not all right angles and any per-degree scale error
        // shows up at a different magnitude.
        maneuvers.add(new Maneuver(
                "OB_diff_triangle",
                "Equilateral triangle with 5 ft sides and 120 deg turns",
                List.of(
                        Leg.drive(FIVE_FEET_METERS), Leg.turn(120),
                        Leg.drive(FIVE_FEET_METERS), Leg.turn(120),
                        Leg.drive(FIVE_FEET_METERS), Leg.turn(120)),
                true));

        return maneuvers;
    }

    /**
     * Every manoeuvre in the catalogue.
     *
     * @return permutations, then same-path returns, then different-path returns.
     */
    public static List<Maneuver> all() {
        List<Maneuver> maneuvers = new ArrayList<>();
        maneuvers.addAll(permutations());
        maneuvers.addAll(outAndBackSamePath());
        maneuvers.addAll(outAndBackDifferentPath());
        return maneuvers;
    }

    /**
     * Total driving distance across a set of manoeuvres, to judge how much space and time a
     * session needs.
     *
     * @param maneuvers Manoeuvres to total.
     * @return combined distance in metres.
     */
    public static double totalDistanceMeters(List<Maneuver> maneuvers) {
        double total = 0;
        for (Maneuver maneuver : maneuvers) {
            total += maneuver.totalDistanceMeters();
        }
        return total;
    }
}
