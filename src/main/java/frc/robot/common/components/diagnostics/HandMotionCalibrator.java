package frc.robot.common.components.diagnostics;

/**
 * Works out encoder polarity and arm travel from a mechanism moved <b>by hand</b>, motors unpowered.
 *
 * <p>Every powered test in the calibration sequence assumes it already knows which way is forward. A
 * sign error makes the drive characterisation fit a negative gain, sends the arm's profile away from
 * its goal until it hits steel, and makes a swerve module servo to the long way round for ever. Those
 * all present as mechanical faults, and the shop spends the afternoon on the mechanism.
 *
 * <p>Establishing polarity by hand first is what makes the rest of the sequence safe, and it is worth
 * doing in this order for a reason that is not obvious: <b>the cheapest way to find a sign error is
 * with the motors off.</b> A powered direction test discovers an inverted arm by driving it into its
 * own hard stop at full command. Turning it by hand discovers the same thing at zero risk, and takes
 * about fifteen seconds per mechanism.
 *
 * <h2>What "positive" means</h2>
 *
 * <p>The operator is asked to move the mechanism the way the <em>code</em> calls positive — the way
 * the robot would move forward, the way the arm deploys. The encoder then either agrees or it does
 * not. Nothing here knows about wiring; it reports the disagreement and what to change.
 *
 * <h2>Why net travel is not enough on its own</h2>
 *
 * <p>A hand pushing a wheel wobbles, and a hand pushing an arm through its arc pauses and backs off.
 * A test that only looked at the net change could read a clean push and a wobble that happened to end
 * slightly forward as the same result. So absolute travel is accumulated alongside net travel, and a
 * motion where most of the movement cancelled itself out is reported as {@link Polarity#AMBIGUOUS}
 * rather than resolved in favour of whichever way the last wobble went.
 *
 * <p>Pure arithmetic, so all of it is directly testable.
 */
public class HandMotionCalibrator {

    /**
     * Rotations of absolute travel below which a step is not a measurement.
     *
     * <p>Guards the two ways a step can produce nothing: the operator pressed the button without
     * moving anything, and the encoder is dead or reading a different device. Both would otherwise
     * yield a confident polarity from noise.
     */
    public static final double MIN_TRAVEL_ROTATIONS = 0.25;

    /**
     * Least share of the total motion that must be in one direction.
     *
     * <p>At 0.5, half the hand's movement may be slop and the answer still stands. Below that, the
     * operator moved it back and forth about as much as forward, and no direction was demonstrated.
     */
    public static final double MIN_DIRECTIONAL_SHARE = 0.5;

    /** How an encoder's sign compares with the direction the operator was asked to move. */
    public enum Polarity {
        /** Encoder went positive when moved the way the code calls positive. Nothing to change. */
        AGREES,
        /** Encoder went negative. The motor or its encoder is inverted relative to the code. */
        INVERTED,
        /** Nothing moved far enough to tell. Either it was not moved, or the encoder is not reading. */
        NO_MOTION,
        /** It moved, but roughly as much backwards as forwards. No direction was demonstrated. */
        AMBIGUOUS
    }

    /** Accumulates one mechanism's hand motion. */
    public static class Motion {

        private final String mechanism;
        private final String positiveDirection;

        private double net;
        private double travel;
        private int samples;
        private double lastReading;
        private boolean started;

        /**
         * @param mechanism         Human name, e.g. "FRONT-LEFT DRIVE".
         * @param positiveDirection What the operator was asked to do, e.g. "roll so the robot drives
         *                          FORWARD". Carried through to the report so a result can never be
         *                          read without knowing which direction produced it.
         */
        public Motion(String mechanism, String positiveDirection) {
            this.mechanism = mechanism;
            this.positiveDirection = positiveDirection;
        }

        /**
         * Folds in a position reading.
         *
         * <p>The first reading establishes the reference without accumulating, so a routine that
         * starts with the encoder at 8.0 rotations does not record an 8.0-rotation jump from zero.
         *
         * @param reading Current position, in whatever unit the mechanism reports.
         */
        public void addPosition(double reading) {
            if (!started) {
                lastReading = reading;
                started = true;
                return;
            }
            addDelta(reading - lastReading);
            lastReading = reading;
        }

        /**
         * Folds in a velocity reading, for mechanisms that report no position.
         *
         * <p>A hand-spun flywheel is measured this way. Integrating velocity rather than differencing
         * position costs nothing here: the direction is in the sign either way, and a slow careful
         * push still accumulates because it is the integral that grows, not the instantaneous value.
         *
         * @param velocity  Current velocity, per second in the same unit as the result.
         * @param seconds   Time since the previous sample.
         */
        public void addVelocity(double velocity, double seconds) {
            addDelta(velocity * seconds);
            started = true;
        }

        private void addDelta(double delta) {
            net += delta;
            travel += Math.abs(delta);
            samples++;
        }

        /** @return net signed movement. */
        public double getNet() {
            return net;
        }

        /** @return total movement regardless of direction. */
        public double getTravel() {
            return travel;
        }

        /** @return samples folded in. */
        public int getSamples() {
            return samples;
        }

        /** @return the verdict for this mechanism. */
        public Result result() {
            return analyse(mechanism, positiveDirection, net, travel, samples);
        }
    }

    /**
     * One mechanism's polarity verdict.
     *
     * @param mechanism         Human name of the mechanism.
     * @param positiveDirection What the operator was asked to do.
     * @param polarity          The verdict.
     * @param net               Net signed movement seen.
     * @param travel            Total movement regardless of direction.
     * @param samples           Samples folded in.
     */
    public record Result(String mechanism, String positiveDirection, Polarity polarity, double net,
            double travel, int samples) {

        /** @return whether this step produced a usable answer. */
        public boolean isConclusive() {
            return polarity == Polarity.AGREES || polarity == Polarity.INVERTED;
        }

        /** @return a report line, including what to change when the encoder disagrees. */
        public String describe() {
            String head = String.format("%s: net %+.2f, travel %.2f over %d samples -> %s",
                    mechanism, net, travel, samples, polarity);

            switch (polarity) {
                case AGREES:
                    return head + "\n      Correct. Nothing to change.";
                case INVERTED:
                    return head + "\n      *** INVERTED *** Asked to " + positiveDirection
                            + ", encoder went the other way."
                            + "\n      Flip the inversion for this motor in its config, then re-run"
                            + " this step to confirm."
                            + "\n      Do NOT run any powered test on this mechanism until it agrees.";
                case NO_MOTION:
                    return head + "\n      NOT MEASURED. Less than " + MIN_TRAVEL_ROTATIONS
                            + " of travel."
                            + "\n      Either it was not moved, it is still in brake mode, or the"
                            + " encoder is not reporting."
                            + "\n      Check the position actually changes on the dashboard as you"
                            + " move it, then re-run.";
                case AMBIGUOUS:
                default:
                    return head + "\n      NOT MEASURED. It moved " + String.format("%.2f", travel)
                            + " but only netted " + String.format("%+.2f", net) + "."
                            + "\n      Move it steadily one way through the whole step instead of"
                            + " back and forth, then re-run.";
            }
        }
    }

    /**
     * Classifies one hand motion.
     *
     * @param mechanism         Human name.
     * @param positiveDirection What the operator was asked to do.
     * @param net               Net signed movement.
     * @param travel            Total movement regardless of direction.
     * @param samples           Samples folded in.
     * @return the verdict.
     */
    public static Result analyse(String mechanism, String positiveDirection, double net,
            double travel, int samples) {

        if (travel < MIN_TRAVEL_ROTATIONS) {
            return new Result(mechanism, positiveDirection, Polarity.NO_MOTION, net, travel, samples);
        }

        // Checked after the travel gate, because a tiny wobble would otherwise be reported as
        // AMBIGUOUS — which reads as "you moved it wrong" when the real answer is "you barely moved
        // it at all", and sends the operator to fix the wrong thing.
        if (Math.abs(net) < MIN_DIRECTIONAL_SHARE * travel) {
            return new Result(mechanism, positiveDirection, Polarity.AMBIGUOUS, net, travel, samples);
        }

        return new Result(mechanism, positiveDirection,
                net > 0 ? Polarity.AGREES : Polarity.INVERTED, net, travel, samples);
    }

    /**
     * An arm's travel, measured by hand between its two physical stops.
     *
     * @param stowedPosition   Encoder reading against the stowed stop.
     * @param deployedPosition Encoder reading against the deployed stop.
     */
    public record ArmTravel(double stowedPosition, double deployedPosition) {

        /** @return signed travel from stowed to deployed. */
        public double travel() {
            return deployedPosition - stowedPosition;
        }

        /**
         * @return the recommended soft limit at the stowed end, inset from the stop.
         *
         *     <p>Inset by {@link #MARGIN_ROTATIONS} because a soft limit exactly at a hard stop is a
         *     soft limit the arm reaches by hitting steel. The inset is what makes the limit do its
         *     job rather than document the crash.
         */
        public double recommendedStowLimit() {
            return stowedPosition + Math.signum(travel()) * MARGIN_ROTATIONS;
        }

        /** @return the recommended soft limit at the deployed end, inset from the stop. */
        public double recommendedDeployLimit() {
            return deployedPosition - Math.signum(travel()) * MARGIN_ROTATIONS;
        }

        /** @return true when the two stops are far enough apart to be two distinct stops. */
        public boolean isUsable() {
            return Math.abs(travel()) >= MIN_TRAVEL_ROTATIONS;
        }

        /**
         * @return true when deploying increases the encoder reading.
         *
         *     <p>Reported rather than assumed. This single fact decides the sign of every soft limit,
         *     goal and profile on the arm, and measuring it by hand is the only way to know it before
         *     something is driven at it.
         */
        public boolean deployIsPositive() {
            return travel() > 0;
        }

        /** Rotations held back from each stop when recommending a soft limit. */
        public static final double MARGIN_ROTATIONS = 0.25;

        /**
         * @param configuredStow   The stow target currently in constants.
         * @param configuredDeploy The deploy target currently in constants.
         * @return a report comparing measured travel with what the code believes.
         */
        public String describe(double configuredStow, double configuredDeploy) {
            if (!isUsable()) {
                return String.format(
                        "ARM TRAVEL: stowed %.2f, deployed %.2f -> only %.2f apart. NOT MEASURED.%n"
                            + "      Both readings came from effectively the same place. Move the arm"
                            + " all the way to each stop%n"
                            + "      and press NEXT only once it is against steel.",
                        stowedPosition, deployedPosition, Math.abs(travel()));
            }

            double configuredTravel = configuredDeploy - configuredStow;
            String directionNote = deployIsPositive()
                    ? "deploying INCREASES the encoder"
                    : "deploying DECREASES the encoder";

            StringBuilder out = new StringBuilder();
            out.append(String.format(
                    "ARM TRAVEL: stowed %.2f, deployed %.2f -> %+.2f rotations (%s)",
                    stowedPosition, deployedPosition, travel(), directionNote));
            out.append(String.format("%n      Recommended soft limits: %.2f to %.2f"
                    + "  (%.2f held back from each stop)",
                    Math.min(recommendedStowLimit(), recommendedDeployLimit()),
                    Math.max(recommendedStowLimit(), recommendedDeployLimit()),
                    MARGIN_ROTATIONS));

            if (Math.signum(travel()) != Math.signum(configuredTravel)) {
                out.append(String.format("%n      *** SIGN MISMATCH *** Constants go %+.2f, the arm"
                        + " goes %+.2f.", configuredTravel, travel()));
                out.append("\n      Every goal on this arm is currently the wrong way round. Fix"
                        + " this before powering it.");
            } else if (Math.abs(Math.abs(travel()) - Math.abs(configuredTravel))
                    > 0.2 * Math.abs(travel())) {
                out.append(String.format("%n      Travel disagrees with constants by more than 20%%:"
                        + " measured %.2f, configured %.2f.", Math.abs(travel()),
                        Math.abs(configuredTravel)));
                out.append("\n      The targets will not reach the ends of travel, or will drive"
                        + " past them into the stops.");
            } else {
                out.append("\n      Agrees with constants to within 20%.");
            }

            out.append("\n      This measurement is ground truth for the powered travel test:"
                    + " it must report the same span.");
            return out.toString();
        }
    }
}
