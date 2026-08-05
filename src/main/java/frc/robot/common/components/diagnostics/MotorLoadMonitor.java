package frc.robot.common.components.diagnostics;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.CommonConstants.BatteryConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Works out what a mechanism is doing from its current draw and its speed.
 *
 * <p>A roller that has just taken in a game piece draws more current. So does one that is jammed.
 * So does one that is simply cold, or running on a fresh battery. <b>Current on its own cannot tell
 * those apart</b>, and a detector built on an absolute current threshold will either miss pieces on
 * a tired battery or cry jam every time the mechanism does its job.
 *
 * <p>Two things make it reliable:
 *
 * <ul>
 *   <li><b>Compare against a learned baseline, not a fixed number.</b> The free-running current of a
 *       roller changes with battery voltage, temperature and wear. The baseline here is learned
 *       continuously while the mechanism is running <em>unloaded</em>, so what is measured is the
 *       <em>excess</em> caused by a game piece rather than the absolute draw.</li>
 *   <li><b>Use velocity to disambiguate.</b> High current with healthy speed means the mechanism is
 *       doing work — a piece is going through. High current with collapsed speed means it is loaded
 *       but not moving product, which is a jam. This single distinction is what separates "ball
 *       ingested" from "ball stuck", and no amount of current filtering achieves it alone.</li>
 * </ul>
 *
 * <p>Thresholds are marked MEASURE and are currently reasoned rather than measured. Every
 * mechanism's current is already logged, so one session with and without a game piece gives the real
 * numbers: watch {@code .../OutputCurrent} while running empty, then with a piece going through, and
 * set the excess threshold between the two.
 */
public class MotorLoadMonitor {

    /** What the mechanism appears to be doing. */
    public enum LoadState {
        /** Not commanded to run. */
        IDLE,
        /** Running, current near baseline — nothing going through. */
        RUNNING_EMPTY,
        /** Elevated current with healthy speed — a game piece is being moved. */
        DOING_WORK,
        /** Elevated current with collapsed speed — loaded but not moving product. */
        JAMMED,
        /** Near-zero speed at high current — hard stop. */
        STALLED
    }

    private final String name;

    /** MEASURE — amps above baseline that count as a game piece passing through. */
    private final double workExcessAmps;

    /** MEASURE — fraction of expected speed below which the mechanism is considered stuck. */
    private final double jamSpeedFraction;

    /**
     * Speed the mechanism runs at unloaded, in whatever unit the velocity is reported in.
     *
     * <p>A supplier rather than a constant because the shooter's expected speed <em>is</em> its
     * current setpoint, which ranges from 1700 to 4500 RPM depending on the shot. A single fixed
     * value would be correct at one setpoint and wrong at the other four — generous enough at 4500
     * to never detect a jam, or tight enough at 1700 to call every shot a jam.
     */
    private final DoubleSupplier expectedSpeed;

    /** Consecutive loops of elevated current and low speed before calling it a jam. */
    private final int jamLoops;

    /**
     * Smooths current before any decision is made.
     *
     * <p>SPARK current readings are noisy enough that a single sample regularly doubles. A 5-sample
     * moving average costs 100 ms of latency, which is nothing against how long a jam persists but
     * removes almost all the false triggering.
     */
    private final LinearFilter currentFilter = LinearFilter.movingAverage(5);

    /**
     * How fast the baseline tracks, per loop.
     *
     * <p>0.01 gives roughly a 100-loop time constant — two seconds. Slow enough that a game piece
     * passing through does not shift it, fast enough to follow battery sag over a match.
     *
     * <p>Deliberately <em>not</em> a {@code LinearFilter.movingAverage}. That ramps up from zero, so
     * for its first window the baseline reads far too low: measured at 4.6 A against a true 8 A idle
     * after 60 samples. Excess is baseline-relative, so an artificially low baseline inflates it and
     * the monitor reports game pieces that are not there for the first seconds of every run. An
     * exponential average seeded from the first real reading starts at the right value and has no
     * warm-up artefact.
     */
    private static final double BASELINE_ALPHA = 0.01;

    /**
     * How fast the learned speed ratio tracks, per loop.
     *
     * <p>Same time constant as the current baseline, and for the same reason.
     */
    private static final double SPEED_RATIO_ALPHA = 0.01;

    /**
     * Bounds on the learned speed ratio.
     *
     * <p>A mechanism running at less than 20% or more than 150% of its nominal expected speed while
     * apparently unloaded means something is wrong with the nominal figure, the gearing, or the
     * reading. Clamping stops one bad sample from moving the jam threshold somewhere it can never
     * fire, or somewhere it fires constantly.
     */
    private static final double MIN_SPEED_RATIO = 0.20;
    private static final double MAX_SPEED_RATIO = 1.50;

    private double filteredCurrent;
    private double baselineCurrent;
    private double lastSpeed;
    private boolean baselineEstablished;
    private int baselineSamples;

    /**
     * Measured unloaded speed as a fraction of the nominal expected speed.
     *
     * <p><b>Why a ratio rather than an absolute learned speed.</b> The nominal figure is a datasheet
     * number, and a real mechanism never reaches it: gearing and belt drag, bearing friction and air
     * resistance all take a cut, and <b>battery voltage falls as current spikes and as charge
     * depletes</b>, which scales speed directly. A jam threshold set as a fraction of the datasheet
     * figure therefore drifts closer to a healthy mechanism's real speed as the match goes on — so
     * late in a match, on a tired pack, a perfectly good roller starts reading as jammed.
     *
     * <p>Learning the ratio absorbs all of it. It is dimensionless, which is what makes it work for
     * the flywheel too: there the nominal figure is the live setpoint, so an absolute learned speed
     * would be stale the moment the operator changed preset and a drop from the corner shot to the
     * hub shot would read as a jam. A ratio stays near 1.0 across every setpoint because the closed
     * loop tracks whatever it is given.
     *
     * <p>Starts at 1.0, so before anything is learned the behaviour is exactly the old behaviour.
     */
    private double speedRatio = 1.0;
    private int elevatedLowSpeedLoops;
    private LoadState state = LoadState.IDLE;

    /**
     * @param name             Log prefix, e.g. "Intake/Rollers".
     * @param workExcessAmps   MEASURE — amps above baseline indicating a piece is being moved.
     * @param expectedSpeed    Unloaded running speed, same unit as the velocity supplied.
     * @param jamSpeedFraction MEASURE — below this fraction of expected speed, treat as stuck.
     * @param jamLoops         Consecutive qualifying loops before declaring a jam.
     */
    public MotorLoadMonitor(String name, double workExcessAmps, double expectedSpeed,
            double jamSpeedFraction, int jamLoops) {
        this(name, workExcessAmps, () -> expectedSpeed, jamSpeedFraction, jamLoops);
    }

    /**
     * As above, but for a mechanism whose unloaded speed depends on its setpoint.
     *
     * @param name             Log prefix, e.g. "Shooter/Flywheel".
     * @param workExcessAmps   MEASURE — amps above baseline indicating a piece is being moved.
     * @param expectedSpeed    Supplies the speed the mechanism should currently be running at.
     * @param jamSpeedFraction MEASURE — below this fraction of expected speed, treat as stuck.
     * @param jamLoops         Consecutive qualifying loops before declaring a jam.
     */
    public MotorLoadMonitor(String name, double workExcessAmps, DoubleSupplier expectedSpeed,
            double jamSpeedFraction, int jamLoops) {
        this.name = name;
        this.workExcessAmps = workExcessAmps;
        this.expectedSpeed = expectedSpeed;
        this.jamSpeedFraction = jamSpeedFraction;
        this.jamLoops = jamLoops;
    }

    /** @return the speed the mechanism is currently expected to run at, always non-negative. */
    private double expectedSpeed() {
        return Math.abs(expectedSpeed.getAsDouble());
    }

    /**
     * Folds in one loop's readings.
     *
     * @param amps      Motor output current.
     * @param speed     Mechanism speed, any consistent unit matching {@code expectedSpeed}.
     * @param commanded Whether the mechanism is currently commanded to run.
     * @return the state this loop.
     */
    public LoadState update(double amps, double speed, boolean commanded) {
        filteredCurrent = currentFilter.calculate(amps);
        lastSpeed = speed;

        if (!commanded) {
            // A stopped mechanism tells us nothing about its loaded or unloaded current, and
            // folding zeros into the baseline would drag it towards zero and make every
            // subsequent run look loaded.
            elevatedLowSpeedLoops = 0;
            state = LoadState.IDLE;
            log();
            return state;
        }

        // Bus voltage first, because speed scales with it almost linearly and this robot runs
        // anywhere from 6 to 16 V. A mechanism on a 10 V pack is genuinely slower than the same
        // mechanism on 13 V, and without this the jam threshold would call the difference a fault.
        //
        // Layered deliberately: physics handles the FAST changes, learning handles the slow ones. Sag
        // under load happens in milliseconds, far quicker than the two-second time constant on the
        // learned ratio could follow, so it has to be computed rather than learned. Drag and wear are
        // the opposite, and are left to the ratio.
        double busVolts = RobotController.getBatteryVoltage();
        double voltageScale = busVolts > 1.0
                ? busVolts / BatteryConstants.NOMINAL_VOLTS
                : 1.0;

        double nominalExpected = expectedSpeed() * voltageScale;

        // The speed a healthy mechanism actually reaches, rather than the one the datasheet
        // promises. This is what the jam threshold is a fraction of.
        double expected = nominalExpected * speedRatio;

        double excess = filteredCurrent - baselineCurrent;
        boolean elevated = baselineEstablished && excess >= workExcessAmps;
        boolean speedCollapsed = expected > 0
                && Math.abs(speed) < expected * jamSpeedFraction;

        // Only learn while running and apparently unloaded, so a game piece does not teach the
        // monitor that loaded current is normal or that a bogged-down speed is healthy.
        if (!elevated) {
            if (baselineSamples == 0) {
                // Seed from the first raw reading rather than from zero. Starting at zero and
                // ramping is what produced false detections during warm-up.
                baselineCurrent = amps;
            } else {
                baselineCurrent += BASELINE_ALPHA * (filteredCurrent - baselineCurrent);
            }

            // Learn the speed ratio on the same gate. Skipped when the nominal figure is zero or
            // near it — a stopped flywheel would otherwise produce a meaningless or infinite ratio.
            if (nominalExpected > 1e-6) {
                double observedRatio = Math.abs(speed) / nominalExpected;
                if (baselineSamples == 0) {
                    speedRatio = observedRatio;
                } else {
                    speedRatio += SPEED_RATIO_ALPHA * (observedRatio - speedRatio);
                }
                speedRatio = Math.max(MIN_SPEED_RATIO, Math.min(MAX_SPEED_RATIO, speedRatio));
            }

            baselineSamples++;
            if (baselineSamples >= 25) {
                baselineEstablished = true;
            }
        }

        if (elevated && speedCollapsed) {
            elevatedLowSpeedLoops++;
        } else {
            elevatedLowSpeedLoops = 0;
        }

        if (elevatedLowSpeedLoops >= jamLoops) {
            // Distinguish a total stop from merely bogged down: both need clearing, but a stall is
            // the more urgent and warrants backing off rather than jostling harder.
            state = Math.abs(speed) < expected * 0.05
                    ? LoadState.STALLED
                    : LoadState.JAMMED;
        } else if (elevated) {
            state = LoadState.DOING_WORK;
        } else {
            state = LoadState.RUNNING_EMPTY;
        }

        log();
        return state;
    }

    private void log() {
        String root = "Load/" + name;
        Logger.recordOutput(root + "/State", state.name());
        Logger.recordOutput(root + "/FilteredAmps", filteredCurrent);
        Logger.recordOutput(root + "/BaselineAmps", baselineCurrent);
        Logger.recordOutput(root + "/ExcessAmps", filteredCurrent - baselineCurrent);
        Logger.recordOutput(root + "/Speed", lastSpeed);
        Logger.recordOutput(root + "/BaselineEstablished", baselineEstablished);
        Logger.recordOutput(root + "/SpeedRatio", speedRatio);
        Logger.recordOutput(root + "/EffectiveExpectedSpeed", getEffectiveExpectedSpeed());

        // Bus voltage, because it is the usual reason a mechanism is slower today than yesterday.
        // Logged here rather than left to be correlated by hand across two different log trees.
        Logger.recordOutput(root + "/BusVolts", RobotController.getBatteryVoltage());
    }

    /** @return the state from the most recent update. */
    public LoadState getState() {
        return state;
    }

    /** @return true while a game piece appears to be moving through. */
    public boolean isDoingWork() {
        return state == LoadState.DOING_WORK;
    }

    /** @return true when the mechanism is loaded but not moving product. */
    public boolean isJammed() {
        return state == LoadState.JAMMED || state == LoadState.STALLED;
    }

    /** @return smoothed current in amps. */
    public double getFilteredCurrent() {
        return filteredCurrent;
    }

    /**
     * @return measured unloaded speed as a fraction of the nominal expected speed.
     *
     *     <p>A value well below 1.0 is not a fault — it is drag, friction and battery voltage,
     *     which is exactly what it exists to absorb. It is worth watching though: a ratio that
     *     falls steadily over a season is a mechanism binding up.
     */
    public double getSpeedRatio() {
        return speedRatio;
    }

    /**
     * @return the speed the jam threshold is actually a fraction of, in the supplied unit.
     *
     *     <p>Nominal expected speed, scaled for the present bus voltage, then scaled by the learned
     *     ratio. All three factors matter: the datasheet figure, what the battery can currently
     *     deliver, and what this particular mechanism actually manages.
     */
    public double getEffectiveExpectedSpeed() {
        double busVolts = RobotController.getBatteryVoltage();
        double voltageScale = busVolts > 1.0 ? busVolts / BatteryConstants.NOMINAL_VOLTS : 1.0;
        return expectedSpeed() * voltageScale * speedRatio;
    }

    /** @return learned unloaded current in amps. */
    public double getBaselineCurrent() {
        return baselineCurrent;
    }

    /** @return current above the learned baseline, in amps. */
    public double getExcessCurrent() {
        return filteredCurrent - baselineCurrent;
    }

    /**
     * @return true once enough unloaded samples have been seen for detection to mean anything.
     *     Before this, {@link #isDoingWork()} and {@link #isJammed()} stay false rather than
     *     guessing from an unlearned baseline.
     */
    public boolean isBaselineEstablished() {
        return baselineEstablished;
    }

    /** Forgets the learned baseline. Use after changing gearing, a motor, or a game piece type. */
    public void reset() {
        currentFilter.reset();
        filteredCurrent = 0;
        baselineCurrent = 0;
        baselineEstablished = false;
        baselineSamples = 0;
        elevatedLowSpeedLoops = 0;
        speedRatio = 1.0;
        state = LoadState.IDLE;
    }

    /** @return this monitor's log prefix. */
    public String getName() {
        return name;
    }
}
