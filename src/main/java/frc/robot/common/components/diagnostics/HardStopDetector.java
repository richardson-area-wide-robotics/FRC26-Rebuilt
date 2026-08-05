package frc.robot.common.components.diagnostics;

import org.littletonrobotics.junction.Logger;

/**
 * Tells a mechanism reaching its physical end stop apart from one hitting a game piece.
 *
 * <p>The intake deploy arm swings between two hard stops, and it also swings up to shake jammed balls
 * loose — so it meets both a steel stop and a ball, and the current signature alone does not
 * distinguish them. Both look like "current up, speed down", which is all
 * {@link MotorLoadMonitor} can see.
 *
 * <h2>What actually separates them</h2>
 *
 * <p><b>A hard stop does not move. A ball does.</b> That is the whole discriminator, and it is
 * mechanical rather than statistical:
 *
 * <ul>
 *   <li>A steel stop with the motor pushing into it holds position to within encoder noise,
 *       indefinitely. It never yields.
 *   <li>A ball squashes, rolls, or squirts out of the way. Position keeps <em>creeping</em>, and
 *       usually the obstruction clears within a few hundred milliseconds.
 * </ul>
 *
 * <p>So the test is not the current level, it is <b>whether position is frozen or creeping while the
 * motor is pushing</b>. Current only says something is being pushed against.
 *
 * <h2>The third case, which current gets backwards</h2>
 *
 * <p>A <b>soft limit</b> also stops the mechanism dead with position frozen — but the SPARK enforces
 * it by cutting output, so <b>current falls to nothing</b> rather than rising. A detector looking only
 * for "stopped" would call that a hard stop and would then happily re-zero the encoder against a
 * limit that is itself defined in encoder units. Circular, and self-reinforcing once wrong.
 *
 * <p>So: frozen with high current is a hard stop, frozen with no current is a soft limit, and the two
 * must not be confused.
 *
 * <h2>Why this is worth more than just knowing the arm arrived</h2>
 *
 * <p>{@code Intake} calls {@code deployEncoder.setPosition(0)} in its constructor, which assumes the
 * arm is stowed when the code starts. If it ever boots part-way — a mid-match reboot, a brownout, or
 * someone moving it by hand with the robot off — every position afterwards is offset by that error,
 * and <b>the soft limits are wrong by the same amount</b> because they are expressed in the same
 * units. Soft limits referenced to a relative encoder are only as trustworthy as the boot assumption.
 *
 * <p>A confirmed hard stop is an absolute reference. {@link #getLearnedStop} exposes it so the encoder
 * can be re-zeroed against something physical rather than against an assumption.
 */
public class HardStopDetector {

    /** What the mechanism appears to be doing at the end of its travel. */
    public enum State {
        /** Not being commanded anywhere. */
        IDLE,
        /** Moving normally. */
        MOVING,
        /** Pushing, position frozen, current high — the physical end of travel. */
        AT_HARD_STOP,
        /** Pushing, position creeping — something soft and movable is in the way. */
        OBSTRUCTED,
        /**
         * Position frozen but current is near zero, so the controller has cut output.
         *
         * <p>A soft limit, not a stop. Must not be treated as a position reference: the limit is
         * itself defined in encoder units, so re-zeroing against it would be circular.
         */
        AT_SOFT_LIMIT
    }

    /** Which end of travel. */
    public enum End {
        /** The negative-position end. */
        LOW,
        /** The positive-position end. */
        HIGH
    }

    private final String name;

    /**
     * MEASURE — position range, in mechanism units, below which position counts as frozen.
     *
     * <p>Judged over {@link #sustainLoops}, not per sample. A steel stop holds to encoder noise,
     * which on a SPARK relative encoder is a few thousandths of a rotation. A ball being compressed
     * yields tenths. 0.05 sits an order of magnitude above the noise and an order below the yield.
     */
    private final double frozenBand;

    /** MEASURE — amps above which the motor counts as genuinely pushing rather than coasting. */
    private final double pushingAmps;

    /** Consecutive qualifying loops before a verdict. */
    private final int sustainLoops;

    /** Rolling window of positions, to judge frozen against creeping. */
    private final double[] window;
    private int windowFill;
    private int windowNext;

    private final VisionCalibration.RunningStats lowStop = new VisionCalibration.RunningStats();
    private final VisionCalibration.RunningStats highStop = new VisionCalibration.RunningStats();

    private State state = State.IDLE;
    private int qualifyingLoops;

    /** Sign of the last non-zero command, for spotting a reversal. */
    private double lastDirection;

    /** Consecutive loops of intent with no output, for spotting a soft limit. */
    private int softLimitLoops;
    private double lastPosition;
    private double lastAmps;
    private double lastPositionSpread;

    /**
     * @param name         Log prefix, e.g. "Intake/Deploy".
     * @param frozenBand   MEASURE — position range counting as frozen, in mechanism units.
     * @param pushingAmps  MEASURE — current above which the motor is really pushing.
     * @param sustainLoops Loops the condition must hold. 12 loops is 240 ms.
     */
    public HardStopDetector(String name, double frozenBand, double pushingAmps, int sustainLoops) {
        this.name = name;
        this.frozenBand = frozenBand;
        this.pushingAmps = pushingAmps;
        this.sustainLoops = sustainLoops;
        this.window = new double[Math.max(2, sustainLoops)];
    }

    /**
     * Folds in one loop.
     *
     * @param position          Mechanism position.
     * @param amps              Motor current.
     * @param commandedOutput   What the motor was told to do this loop. Sign gives the direction.
     * @return the state this loop.
     */
    public State update(double position, double amps, double commandedOutput) {
        return update(position, amps, commandedOutput, commandedOutput);
    }

    /**
     * Folds in one loop, distinguishing what was asked for from what came out.
     *
     * <p>The two differ in exactly one interesting case, and it is the case
     * {@link State#AT_SOFT_LIMIT} exists for: a controller enforces a soft limit by <b>zeroing its
     * own output</b>. So intent is non-zero while applied output is zero, which is a far more direct
     * signal than the one this used to infer from frozen position and falling current.
     *
     * <p>It also has to be this way round now that the arm's profile runs on the SPARK. Robot code no
     * longer writes an output during a profiled move, so the only faithful account of what is coming
     * out of the controller is the controller's own applied output -- while the only account of what
     * was <em>wanted</em> is still robot-side. Passing applied output for both, as this briefly did,
     * made AT_SOFT_LIMIT unreachable: a soft limit zeroes the output, so the near-zero check below
     * classified it as IDLE before anything else was considered.
     *
     * @param position       Mechanism position.
     * @param amps           Motor current.
     * @param intendedOutput What the mechanism was asked to do. Sign gives the direction.
     * @param appliedOutput  What the controller is actually applying.
     * @return the state this loop.
     */
    public State update(double position, double amps, double intendedOutput,
            double appliedOutput) {

        // Intent without output, sustained, is a soft limit cutting in. Checked before the idle gate,
        // because the idle gate is written in terms of output and would otherwise absorb this.
        if (Math.abs(intendedOutput) >= 0.02 && Math.abs(appliedOutput) < 0.02) {
            lastPosition = position;
            lastAmps = amps;
            softLimitLoops++;
            if (softLimitLoops >= sustainLoops) {
                state = State.AT_SOFT_LIMIT;
                log();
                return state;
            }
        } else {
            softLimitLoops = 0;
        }

        return updateFromApplied(position, amps, appliedOutput);
    }

    private State updateFromApplied(double position, double amps, double commandedOutput) {
        lastPosition = position;
        lastAmps = amps;

        // A reversal invalidates everything gathered so far, and forgetting that mislabels the stop
        // the arm is leaving as the stop at the OTHER end.
        //
        // How: on the loop the command flips sign, the position window is still full of frozen
        // samples from resting against the stop, qualifyingLoops is already past sustainLoops, and
        // reversal current spikes instantly. So frozen && pushing is satisfied on the very first
        // loop of the new direction, and the position gets filed under the new direction's end.
        //
        // What that cost: DeployTravelCalibrator seeks one stop, reverses, and immediately records
        // the SAME position as the opposite stop. Travel comes out near zero, and
        // Intake.clampToLearnedStops then pins every profiled goal to that one position -- so the
        // arm silently stops responding to stow and deploy alike, having reported a successful
        // calibration.
        double direction = Math.signum(commandedOutput);
        if (direction != 0 && lastDirection != 0 && direction != lastDirection) {
            windowFill = 0;
            qualifyingLoops = 0;
        }
        if (direction != 0) {
            lastDirection = direction;
        }

        // A near-zero command is not evidence of anything. Note the deploy arm's idle is a small
        // negative bias rather than zero, so the threshold has to sit under that or the arm would
        // look idle while it is in fact pressed against its stow stop.
        if (Math.abs(commandedOutput) < 0.02) {
            reset();
            state = State.IDLE;
            log();
            return state;
        }

        pushWindow(position);
        lastPositionSpread = windowSpread();

        boolean frozen = windowFill >= window.length && lastPositionSpread <= frozenBand;
        boolean pushing = amps >= pushingAmps;

        if (!frozen) {
            // Still moving, or creeping. Creeping while pushing is a ball yielding, which is not the
            // end of travel however much current it draws.
            qualifyingLoops = 0;
            state = pushing && windowFill >= window.length ? State.OBSTRUCTED : State.MOVING;
            log();
            return state;
        }

        qualifyingLoops++;
        if (qualifyingLoops < sustainLoops) {
            log();
            return state;
        }

        if (pushing) {
            state = State.AT_HARD_STOP;
            // Learn where the stop is, per direction. Once known it becomes an absolute reference the
            // relative encoder can be corrected against.
            if (commandedOutput > 0) {
                highStop.add(position);
            } else {
                lowStop.add(position);
            }
        } else {
            // Frozen with no current: the controller cut output. A soft limit, not a stop, and
            // deliberately NOT learned as a position reference.
            state = State.AT_SOFT_LIMIT;
        }

        log();
        return state;
    }

    private void pushWindow(double position) {
        window[windowNext] = position;
        windowNext = (windowNext + 1) % window.length;
        if (windowFill < window.length) {
            windowFill++;
        }
    }

    private double windowSpread() {
        if (windowFill == 0) {
            return 0;
        }
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < windowFill; i++) {
            min = Math.min(min, window[i]);
            max = Math.max(max, window[i]);
        }
        return max - min;
    }

    /** @return the state from the most recent update. */
    public State getState() {
        return state;
    }

    /** @return true when the mechanism is against its physical end of travel. */
    public boolean isAtHardStop() {
        return state == State.AT_HARD_STOP;
    }

    /**
     * @return true when something soft and movable is in the way.
     *
     *     <p>On the intake arm this means a ball, which is exactly the case that must not be reported
     *     as "fully deployed".
     */
    public boolean isObstructed() {
        return state == State.OBSTRUCTED;
    }

    /** @return true when the controller has cut output at a soft limit. */
    public boolean isAtSoftLimit() {
        return state == State.AT_SOFT_LIMIT;
    }

    /**
     * @param end Which end of travel.
     * @return the learned stop position, or NaN if that end has not been reached yet.
     */
    public double getLearnedStop(End end) {
        VisionCalibration.RunningStats stats = end == End.LOW ? lowStop : highStop;
        return stats.getCount() > 0 ? stats.getMean() : Double.NaN;
    }

    /**
     * @param end Which end.
     * @return spread of the learned stop position, in mechanism units.
     *
     *     <p>Small spread means the stop is repeatable and the encoder is not slipping. Growing spread
     *     means either the encoder is losing count or the stop itself is moving — a fastener backing
     *     out, say. Worth watching over a season.
     */
    public double getStopSpread(End end) {
        VisionCalibration.RunningStats stats = end == End.LOW ? lowStop : highStop;
        return stats.getCount() > 1 ? stats.getMax() - stats.getMin() : 0;
    }

    /** @param end Which end. @return how many times that stop has been confirmed. */
    public int getStopHits(End end) {
        return (end == End.LOW ? lowStop : highStop).getCount();
    }

    /**
     * @return measured travel between the two stops, or NaN until both have been reached.
     *
     *     <p>This is what validates the deploy target and the soft limits: they are currently
     *     hand-chosen numbers in encoder units, and this is the same quantity measured.
     */
    public double getMeasuredTravel() {
        double low = getLearnedStop(End.LOW);
        double high = getLearnedStop(End.HIGH);
        return Double.isNaN(low) || Double.isNaN(high) ? Double.NaN : high - low;
    }

    /**
     * @param end        Which stop the mechanism is currently against.
     * @param expectedAt What that stop's position is supposed to be.
     * @return how far the encoder has drifted, or NaN if not currently at that stop.
     *
     *     <p>Only meaningful while {@link #isAtHardStop()}. A non-zero value means the encoder's zero
     *     no longer matches the physical stop — most often because the code started with the arm not
     *     where the constructor assumed.
     */
    public double getEncoderDrift(End end, double expectedAt) {
        if (!isAtHardStop()) {
            return Double.NaN;
        }
        return lastPosition - expectedAt;
    }

    /** Forgets everything, including the learned stops. Use after a mechanical change. */
    public void reset() {
        qualifyingLoops = 0;
        windowFill = 0;
        windowNext = 0;
        lastPositionSpread = 0;
    }

    /** Forgets the learned stop positions as well as the current state. */
    public void resetLearned() {
        reset();
        lowStop.reset();
        highStop.reset();
        state = State.IDLE;
    }

    private void log() {
        String root = "HardStop/" + name;
        Logger.recordOutput(root + "/State", state.name());
        Logger.recordOutput(root + "/Position", lastPosition);
        Logger.recordOutput(root + "/Amps", lastAmps);
        Logger.recordOutput(root + "/PositionSpread", lastPositionSpread);
        Logger.recordOutput(root + "/LowStop", getLearnedStop(End.LOW));
        Logger.recordOutput(root + "/HighStop", getLearnedStop(End.HIGH));
        Logger.recordOutput(root + "/LowStopHits", getStopHits(End.LOW));
        Logger.recordOutput(root + "/HighStopHits", getStopHits(End.HIGH));
        Logger.recordOutput(root + "/MeasuredTravel", getMeasuredTravel());
    }

    /** @return this detector's log prefix. */
    public String getName() {
        return name;
    }
}
