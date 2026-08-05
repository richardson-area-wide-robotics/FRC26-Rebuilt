package frc.robot.common.components.diagnostics;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.components.diagnostics.HardStopDetector.End;
import frc.robot.CommonConstants.BatteryConstants;
import frc.robot.rebuilt.RebuiltConstants.IntakeConstants;
import frc.robot.rebuilt.subsystems.Intake;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Measures the numbers the intake arm's trapezoid profile depends on.
 *
 * <p>A profile is only as good as its constraints. If it asks for more velocity or acceleration than
 * the arm can deliver, the controller saturates, the arm falls behind its own setpoint, and the
 * following error grows until the move ends — <b>which looks exactly like a badly tuned gain and is
 * not.</b> No amount of PID tuning fixes a profile that is asking for the impossible.
 *
 * <p>So the most useful thing here is not the feedforward gains. It is the answer to
 * <b>"are the configured limits achievable?"</b>
 *
 * <h2>Four phases</h2>
 *
 * <ol>
 *   <li><b>Break-away voltage, both directions.</b> Ramp from zero until the arm moves. Done both ways
 *       because gravity helps one direction and opposes the other, so an arm has two different
 *       break-away voltages and a single kS is an average of two unlike things.
 *   <li><b>Velocity feedforward.</b> A few voltage steps, each held to steady state, fitted for kV.
 *   <li><b>Holding voltage across the travel.</b> What it takes to hold the arm still at several
 *       positions. The <em>spread</em> across positions is the gravity signature.
 *   <li><b>Achievable limits.</b> A full-output move, measuring peak velocity and peak acceleration.
 *       This is what validates the profile constraints.
 * </ol>
 *
 * <p><b>Run on blocks with no game pieces.</b> The arm swings its full travel repeatedly.
 *
 * <h2>Why gravity is measured as a table rather than fitted to a cosine</h2>
 *
 * <p>The textbook arm feedforward is {@code kG * cos(angle from horizontal)}. Fitting that needs the
 * arm's angle, which needs both the deploy reduction and where horizontal falls on the encoder —
 * neither of which is known on this robot, and the reduction cannot be measured, only counted from CAD.
 *
 * <p>Rather than guess an angle, this measures the holding voltage <em>at positions</em> and reports
 * them. If it barely varies across the travel, a constant bias is sufficient and
 * {@code DEPLOY_HOLD_SPEED} already does that job. If it varies a lot, gravity genuinely matters and
 * the geometry is worth chasing — and the numbers here say how much is at stake before anyone spends
 * time on it.
 */
public class ArmProfileCalibrator {

    /** Volts per second for the break-away ramp. Slow, so the threshold is found precisely. */
    private static final double BREAKAWAY_RAMP_VOLTS_PER_SEC = 0.5;

    /** Seconds to ramp before giving up on finding break-away. */
    private static final double BREAKAWAY_TIMEOUT = 8.0;

    /** Rotations per second above which the arm counts as having broken away. */
    private static final double BREAKAWAY_VELOCITY_RPS = 0.5;

    /** Voltages held to steady state for the kV fit. */
    private static final double[] VELOCITY_STEPS = {1.0, 1.5, 2.0, 2.5, 3.0};

    /**
     * Seconds to hold each velocity step.
     *
     * <p>0.20, not 0.35. Sized against the travel rather than picked: at the 3 V top step the arm runs
     * near 24 rot/s, so 0.35 s covers about 8.3 of roughly 10 rotations of travel — the arm would reach
     * its hard stop <em>during</em> the step and the velocity read at the end would be a deceleration
     * into steel rather than a steady state. 0.20 s covers 4.7 rotations, which leaves room.
     *
     * <p>Still several time constants for a light arm, so steady state is reached. A step that runs
     * into something anyway is discarded rather than trusted -- see {@link #velocityStep}, and note
     * that the discard test had to change: the hard-stop detector needs longer to agree than this step
     * lasts, so the check that actually fires in time is the velocity one.
     */
    private static final double STEP_SECONDS = 0.20;

    /**
     * Velocity below which the arm counts as stalled rather than slow, in rotations per second.
     *
     * <p>Well under the slowest step's expected velocity and well over encoder noise. Its whole job is
     * to notice, <em>without waiting for a detector to agree</em>, that several volts produced no
     * motion -- which means the reading is a stall or a deceleration and not a steady state.
     */
    private static final double STALLED_RPS = 0.05;

    /** Fractions of travel at which holding voltage is measured. */
    private static final double[] HOLD_FRACTIONS = {0.1, 0.3, 0.5, 0.7, 0.9};

    /** Output for the achievable-limits move. Not full, to leave the controller headroom. */
    private static final double LIMIT_TEST_OUTPUT = 0.85;

    /** Seconds of rest between phases. */
    private static final double REST_SECONDS = 0.8;

    private final Intake intake;

    /** Whether the current break-away ramp actually saw the arm move. */
    private boolean brokeAway;

    private double breakawayUp = Double.NaN;
    private double breakawayDown = Double.NaN;
    private final DriveCharacterization.FeedforwardFit velocityFit =
            new DriveCharacterization.FeedforwardFit();
    private final List<double[]> holdSamples = new ArrayList<>();
    private double peakVelocity;
    private double peakAcceleration;

    /** Bus voltage during the limits test, so the reader knows what conditions produced it. */
    private double limitTestVolts;

    private double rampStart;
    private double lastVelocity;
    private double lastVelocityTime;

    /**
     * @param intake The intake, or null to skip.
     */
    public ArmProfileCalibrator(Intake intake) {
        this.intake = intake;
    }

    // ------------------------------------------------------------------ phase 1: break-away

    /**
     * Ramps voltage until the arm moves, and records the voltage it moved at.
     *
     * @param deployDirection true to ramp toward deployed, false toward stowed.
     * @return the command.
     */
    private Command breakaway(boolean deployDirection) {
        String label = deployDirection ? "toward deployed" : "toward stowed";
        double sign = deployDirection ? 1 : -1;

        return Commands.sequence(
                Commands.runOnce(() -> {
                    rampStart = Timer.getFPGATimestamp();
                    brokeAway = false;
                    System.out.println("[arm] break-away ramp " + label);
                }),
                Commands.run(() -> {
                    double volts = sign * BREAKAWAY_RAMP_VOLTS_PER_SEC
                            * (Timer.getFPGATimestamp() - rampStart);
                    intake.setDeployVoltage(volts);
                }, intake)
                        // The flag is set INSIDE the predicate, which is the only place that knows the
                        // arm actually moved. Without it a timeout is indistinguishable from a
                        // break-away, and reports as one at exactly ramp-rate x timeout -- a suspiciously
                        // round 4.000 V that then gets read as a real measurement, and half the
                        // difference between the two directions gets reported as gravity.
                        .until(() -> {
                            if (Math.abs(intake.getDeployVelocity()) / 60.0
                                    > BREAKAWAY_VELOCITY_RPS) {
                                brokeAway = true;
                                return true;
                            }
                            return false;
                        })
                        .withTimeout(BREAKAWAY_TIMEOUT),
                Commands.runOnce(() -> {
                    double volts = brokeAway
                            ? Math.abs(BREAKAWAY_RAMP_VOLTS_PER_SEC
                                    * (Timer.getFPGATimestamp() - rampStart))
                            : Double.NaN;
                    intake.stopDeploy();

                    if (deployDirection) {
                        breakawayUp = volts;
                    } else {
                        breakawayDown = volts;
                    }

                    if (brokeAway) {
                        System.out.printf("[arm] broke away %s at %.3f V%n", label, volts);
                    } else {
                        System.out.printf("[arm] NOT MEASURED %s -- the ramp reached %.1f V in %.0f s "
                                + "and the arm never moved.%n", label,
                                BREAKAWAY_RAMP_VOLTS_PER_SEC * BREAKAWAY_TIMEOUT, BREAKAWAY_TIMEOUT);
                        System.out.println("[arm]   Most likely a soft limit is cutting output at this "
                                + "end of travel. Check it, then re-run.");
                    }
                }, intake),
                Commands.waitSeconds(REST_SECONDS));
    }

    // ------------------------------------------------------------------ phase 2: kV

    /** Holds one voltage to steady state and folds the result into the fit. */
    private Command velocityStep(double volts) {
        return Commands.sequence(
                Commands.run(() -> intake.setDeployVoltage(volts), intake)
                        .withTimeout(STEP_SECONDS),
                Commands.runOnce(() -> {
                    double rps = Math.abs(intake.getDeployVelocity()) / 60.0;

                    // Three ways a step can fail to be a steady-state reading. The original checked
                    // only the one that cannot fire in time.
                    //
                    //   isAtHardStop() needs the detector to fill a 12-sample window AND then sustain
                    //   12 qualifying loops -- at least 240 ms against a 200 ms step. So the guard the
                    //   docstring relies on never executed, and deceleration-into-steel was fed to the
                    //   fit at the highest voltages, biasing the slope rather than adding scatter.
                    //
                    //   isAtSoftLimit() was not checked at all, and a soft limit inside the travel is
                    //   what the arm reaches FIRST.
                    //
                    //   A velocity of nearly nothing under several volts is a stall however it was
                    //   caused, and needs no detector latency to notice. This is the check that
                    //   actually catches the case in time.
                    boolean hitStop = intake.getDeployStops().isAtHardStop()
                            || intake.getDeployStops().isAtSoftLimit()
                            || rps < STALLED_RPS;
                    intake.stopDeploy();

                    if (hitStop) {
                        // The arm ran out of travel inside the step, so this velocity is a
                        // deceleration into the stop rather than a steady state. Feeding it to the fit
                        // would drag kV low, and it would do so most at the highest voltages — biasing
                        // the slope rather than merely adding scatter.
                        System.out.printf("[arm] %.2f V -> DISCARDED, the arm was not moving freely "
                                + "(read %.2f rot/s). It reached a stop or a soft limit inside the "
                                + "step.%n", volts, rps);
                    } else {
                        velocityFit.add(volts, rps);
                        System.out.printf("[arm] %.2f V -> %.2f rot/s%n", volts, rps);
                    }
                }, intake),
                // Return to the stowed end so the next step has room to run.
                Commands.run(intake::manualReverseDeploy, intake)
                        .until(() -> intake.getDeployStops().isAtHardStop())
                        .withTimeout(3.0),
                Commands.runOnce(intake::stopDeploy, intake),
                Commands.waitSeconds(REST_SECONDS));
    }

    // ------------------------------------------------------------------ phase 3: gravity

    /**
     * Drives to a fraction of the travel, then finds the voltage that holds it there.
     *
     * @param fraction Fraction of measured travel.
     * @return the command.
     */
    private Command holdAt(double fraction) {
        // Captured per call so the recording step can see whether the commanding step actually ran.
        // A bare `return` inside the first lambda exits only the lambda, not the sequence -- so the
        // recording step ran regardless, and every one of the five fractions appended a reading of the
        // PARKED arm at the same position. Spread across the samples was then about zero, and the
        // report concluded "gravity barely varies across the travel, not worth chasing the arm
        // geometry" -- a confident answer closing the exact question the phase exists to open, from a
        // phase in which nothing happened. The isEmpty() guard written to catch this was unreachable.
        boolean[] commanded = new boolean[1];
        double[] goal = new double[1];

        return Commands.sequence(
                Commands.runOnce(() -> {
                    commanded[0] = false;

                    double travel = intake.getDeployStops().getMeasuredTravel();
                    double low = intake.getDeployStops().getLearnedStop(End.LOW);
                    if (Double.isNaN(travel)) {
                        System.out.println("[arm] hold test skipped -- travel not measured. "
                                + "Run the deploy travel calibration first.");
                        return;
                    }

                    goal[0] = low + travel * fraction;
                    intake.deployToGoal(goal[0]);
                    commanded[0] = true;
                }, intake),
                Commands.waitSeconds(1.2),
                Commands.runOnce(() -> {
                    if (!commanded[0]) {
                        return;
                    }

                    // Arriving matters as much as commanding. The goal is silently clamped to the
                    // learned stops, and a weak position gain lets the arm settle short -- either way
                    // the voltage would be gravity at some OTHER position than the one being reported.
                    if (!intake.isDeployAtGoal()) {
                        System.out.printf("[arm] hold sample discarded -- asked for %.2f, settled at "
                                + "%.2f. Gravity here would be recorded against the wrong angle.%n",
                                goal[0], intake.getDeployPosition());
                        return;
                    }

                    // Whatever the controller is applying to hold station IS the holding voltage, so
                    // it can be read rather than searched for.
                    double volts = intake.getDeployHoldVolts();
                    double position = intake.getDeployPosition();
                    holdSamples.add(new double[] {position, volts});
                    System.out.printf("[arm] holding at %.2f rotations takes %.3f V%n",
                            position, volts);
                }, intake),
                Commands.waitSeconds(REST_SECONDS));
    }

    // ------------------------------------------------------------------ phase 4: achievable limits

    /** Drives hard across the travel and records the fastest velocity and acceleration seen. */
    private Command achievableLimits() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    peakVelocity = 0;
                    peakAcceleration = 0;
                    lastVelocity = 0;
                    lastVelocityTime = Timer.getFPGATimestamp();
                    System.out.println("[arm] measuring achievable velocity and acceleration");
                }),
                Commands.run(() -> {
                    // A fraction of what the battery can ACTUALLY supply, not of a nominal 12 V.
                    // Asking for 10.2 V on a 10 V pack simply saturates, and the measured limits would
                    // then be the battery's rather than the arm's — reported as the arm being slower
                    // than it is, on exactly the tired pack where someone is most likely to be
                    // debugging something else.
                    intake.setDeployVoltage(
                        RobotController.getBatteryVoltage() * LIMIT_TEST_OUTPUT);

                    double now = Timer.getFPGATimestamp();
                    double rps = Math.abs(intake.getDeployVelocity()) / 60.0;
                    double dt = now - lastVelocityTime;

                    // Only count SPEEDING UP. Math.abs on the difference made a deceleration count
                    // as an acceleration -- and the largest one in the run is the arm being brought
                    // from full speed to zero by its own limit, in one or two loops. That reported
                    // roughly ten times the real figure, so the achievable-acceleration check always
                    // passed and the profile constraint was never actually validated. An arm silently
                    // lagging its own setpoint all season is precisely what this phase exists to catch.
                    boolean speedingUp = Math.abs(rps) > Math.abs(lastVelocity);

                    if (dt > 1e-3 && dt < 0.1 && speedingUp) {
                        peakAcceleration = Math.max(peakAcceleration,
                                Math.abs(rps - lastVelocity) / dt);
                    }
                    peakVelocity = Math.max(peakVelocity, rps);
                    limitTestVolts = RobotController.getBatteryVoltage();
                    lastVelocity = rps;
                    lastVelocityTime = now;
                }, intake)
                        .until(() -> intake.getDeployStops().isAtHardStop())
                        .withTimeout(3.0),
                Commands.runOnce(intake::stopDeploy, intake),
                Commands.waitSeconds(REST_SECONDS));
    }

    // ------------------------------------------------------------------ the whole thing

    /**
     * All four phases, then the report.
     *
     * @return the command.
     */
    public Command full() {
        if (intake == null) {
            return Commands.runOnce(() -> System.out.println("[arm] skipped, no intake"));
        }

        List<Command> phases = new ArrayList<>();
        phases.add(Commands.runOnce(() -> {
            reset();
            System.out.println("[arm] === Intake arm profile calibration ===");
            System.out.println("[arm] On blocks, NO game pieces. The arm swings its full travel "
                    + "several times.");
            if (Double.isNaN(intake.getDeployStops().getMeasuredTravel())) {
                System.out.println("[arm] NOTE: travel not measured yet — run the deploy travel "
                        + "calibration first, or the gravity phase will be skipped.");
            }
        }));

        phases.add(breakaway(true));
        phases.add(breakaway(false));

        for (double volts : VELOCITY_STEPS) {
            phases.add(velocityStep(volts));
        }

        for (double fraction : HOLD_FRACTIONS) {
            phases.add(holdAt(fraction));
        }

        phases.add(achievableLimits());
        phases.add(Commands.runOnce(this::printReport));

        return Commands.sequence(phases.toArray(new Command[0]))
                .finallyDo(interrupted -> intake.stopDeploy())
                .withName("ArmProfile/Full");
    }

    /**
     * What this run managed to establish, for a caller that has to decide rather than read.
     *
     * @param breakawayMeasuredBothWays Both ramps saw the arm move, so the friction/gravity split is
     *                                  real rather than half a timeout.
     * @param kvFit                     Whether the velocity fit is trustworthy.
     * @param holdSamples               Hold readings that survived the arrived-at-goal check.
     * @param peakVelocityRps           Fastest velocity seen, rotations per second.
     * @param peakAccelRps2             Largest <em>speeding-up</em> acceleration seen.
     */
    public record Outcome(boolean breakawayMeasuredBothWays, boolean kvFit, int holdSamples,
            double peakVelocityRps, double peakAccelRps2) {
    }

    /** @return what this run established. */
    public Outcome outcome() {
        return new Outcome(
                !Double.isNaN(breakawayUp) && !Double.isNaN(breakawayDown),
                velocityFit.fit().isTrustworthy(),
                holdSamples.size(),
                peakVelocity,
                peakAcceleration);
    }

    /** Clears everything measured. */
    public void reset() {
        breakawayUp = Double.NaN;
        breakawayDown = Double.NaN;
        velocityFit.reset();
        holdSamples.clear();
        peakVelocity = 0;
        peakAcceleration = 0;
    }

    /** Prints the measurements and what they say about the configured profile. */
    public void printReport() {
        System.out.println();
        System.out.println("=== INTAKE ARM PROFILE REPORT ===");

        // --- break-away, and what its asymmetry means -------------------------------------------
        System.out.printf("  break-away toward deployed  %.3f V%n", breakawayUp);
        System.out.printf("  break-away toward stowed    %.3f V%n", breakawayDown);

        if (!Double.isNaN(breakawayUp) && !Double.isNaN(breakawayDown)) {
            double asymmetry = Math.abs(breakawayUp - breakawayDown);
            System.out.printf("  difference                  %.3f V%n", asymmetry);
            System.out.println("  ^ this difference IS gravity. A pure friction term would be the same");
            System.out.println("    both ways; gravity helps one direction and opposes the other, so");
            System.out.printf("    kS is about %.3f V and the gravity contribution about %.3f V.%n",
                    (breakawayUp + breakawayDown) / 2, asymmetry / 2);
            System.out.printf("    Suggested: DEPLOY_kS = %.3f%n", (breakawayUp + breakawayDown) / 2);
        }

        // --- kV ---------------------------------------------------------------------------------
        DriveCharacterization.Feedforward ff = velocityFit.fit();
        System.out.println();
        System.out.printf("  velocity fit: kS %.4f  kV %.4f V/(rot/s)  R2 %.4f over %d steps%n",
                ff.kS(), ff.kV(), ff.rSquared(), ff.samples());
        if (ff.isTrustworthy()) {
            System.out.printf("  Suggested: DEPLOY_kV = %.4f%n", ff.kV());
        } else {
            System.out.println("  ^ NOT trustworthy. Too few steps, a poor fit, or a negative kV — the");
            System.out.println("    last means voltage and velocity disagree in sign, which is wiring.");
        }

        // --- gravity across the travel -----------------------------------------------------------
        System.out.println();
        if (holdSamples.isEmpty()) {
            System.out.println("  holding voltages: not measured (travel unknown)");
        } else {
            System.out.println("  holding voltage across the travel:");
            double min = Double.POSITIVE_INFINITY;
            double max = Double.NEGATIVE_INFINITY;
            for (double[] sample : holdSamples) {
                System.out.printf("    %6.2f rotations  %.3f V%n", sample[0], sample[1]);
                min = Math.min(min, sample[1]);
                max = Math.max(max, sample[1]);
            }
            double spread = max - min;
            System.out.printf("  spread %.3f V%n", spread);

            if (spread < 0.3) {
                System.out.println("  ^ small. Gravity barely varies across the travel, so a CONSTANT");
                System.out.println("    bias is enough and DEPLOY_HOLD_SPEED already provides one.");
                System.out.println("    Not worth chasing the arm geometry for a cosine term.");
            } else {
                System.out.println("  ^ LARGE. Gravity varies materially with position, so a constant");
                System.out.println("    bias is wrong at one end. This is the case where DEPLOY_kG and");
                System.out.println("    the arm angle are worth having — which needs the deploy");
                System.out.println("    reduction and DEPLOY_HORIZONTAL_OFFSET_ROTATIONS from CAD.");
            }
        }

        // --- the check that matters most ---------------------------------------------------------
        System.out.println();
        System.out.printf("  ACHIEVABLE  peak velocity %.1f rot/s, peak acceleration %.0f rot/s^2%n",
                peakVelocity, peakAcceleration);
        System.out.printf("              measured at %.1f V of bus, %.0f%% output%n",
                limitTestVolts, LIMIT_TEST_OUTPUT * 100);
        if (limitTestVolts < BatteryConstants.TYPICAL_MIN_VOLTS) {
            System.out.println("              ^ below the usual 10 V, so these limits are pessimistic.");
            System.out.println("                Re-run on a fresher pack before lowering the profile.");
        }
        System.out.printf("  CONFIGURED  max velocity  %.1f rot/s, max acceleration  %.0f rot/s^2%n",
                IntakeConstants.DEPLOY_MAX_VELOCITY_RPS, IntakeConstants.DEPLOY_MAX_ACCEL_RPS2);

        boolean velocityImpossible = peakVelocity > 0
                && IntakeConstants.DEPLOY_MAX_VELOCITY_RPS > peakVelocity;
        boolean accelImpossible = peakAcceleration > 0
                && IntakeConstants.DEPLOY_MAX_ACCEL_RPS2 > peakAcceleration;

        if (velocityImpossible || accelImpossible) {
            System.out.println();
            System.out.println("  ^^ THE PROFILE IS ASKING FOR MORE THAN THE ARM CAN DO.");
            System.out.println("     The controller saturates, the arm falls behind its own setpoint,");
            System.out.println("     and the following error grows through the move. That looks exactly");
            System.out.println("     like a badly tuned gain, and no PID tuning fixes it.");
            System.out.printf("     Lower the constraints to about 80%% of achievable: velocity %.0f, "
                    + "acceleration %.0f.%n", peakVelocity * 0.8, peakAcceleration * 0.8);
        } else if (peakVelocity > 0) {
            double headroom = 100 * (1 - IntakeConstants.DEPLOY_MAX_VELOCITY_RPS / peakVelocity);
            System.out.printf("%n  ^ achievable, with %.0f%% velocity headroom. Good.%n", headroom);
        }

        Logger.recordOutput("ArmProfile/BreakawayUpVolts", breakawayUp);
        Logger.recordOutput("ArmProfile/BreakawayDownVolts", breakawayDown);
        Logger.recordOutput("ArmProfile/kV", ff.kV());
        Logger.recordOutput("ArmProfile/PeakVelocityRps", peakVelocity);
        Logger.recordOutput("ArmProfile/PeakAccelRps2", peakAcceleration);

        System.out.println("=== END ===");
        System.out.println();
    }

    /** @return break-away voltage toward deployed, or NaN. */
    public double getBreakawayUp() {
        return breakawayUp;
    }

    /** @return break-away voltage toward stowed, or NaN. */
    public double getBreakawayDown() {
        return breakawayDown;
    }

    /** @return the fitted velocity feedforward. */
    public DriveCharacterization.Feedforward getVelocityFit() {
        return velocityFit.fit();
    }

    /** @return peak velocity seen at near-full output, rot/s. */
    public double getPeakVelocity() {
        return peakVelocity;
    }

    /** @return peak acceleration seen at near-full output, rot/s². */
    public double getPeakAcceleration() {
        return peakAcceleration;
    }
}
