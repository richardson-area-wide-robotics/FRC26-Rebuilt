package frc.robot.common.components.diagnostics;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.components.diagnostics.HardStopDetector.End;
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

    /** Seconds to hold each step. Long enough to reach steady state, short enough not to hit a stop. */
    private static final double STEP_SECONDS = 0.35;

    /** Fractions of travel at which holding voltage is measured. */
    private static final double[] HOLD_FRACTIONS = {0.1, 0.3, 0.5, 0.7, 0.9};

    /** Output for the achievable-limits move. Not full, to leave the controller headroom. */
    private static final double LIMIT_TEST_OUTPUT = 0.85;

    /** Seconds of rest between phases. */
    private static final double REST_SECONDS = 0.8;

    private final Intake intake;

    private double breakawayUp = Double.NaN;
    private double breakawayDown = Double.NaN;
    private final DriveCharacterization.FeedforwardFit velocityFit =
            new DriveCharacterization.FeedforwardFit();
    private final List<double[]> holdSamples = new ArrayList<>();
    private double peakVelocity;
    private double peakAcceleration;

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
                    System.out.println("[arm] break-away ramp " + label);
                }),
                Commands.run(() -> {
                    double volts = sign * BREAKAWAY_RAMP_VOLTS_PER_SEC
                            * (Timer.getFPGATimestamp() - rampStart);
                    intake.setDeployVoltage(volts);
                }, intake)
                        .until(() -> Math.abs(intake.getDeployVelocity()) / 60.0
                                > BREAKAWAY_VELOCITY_RPS)
                        .withTimeout(BREAKAWAY_TIMEOUT),
                Commands.runOnce(() -> {
                    double volts = Math.abs(BREAKAWAY_RAMP_VOLTS_PER_SEC
                            * (Timer.getFPGATimestamp() - rampStart));
                    intake.stopDeploy();

                    if (deployDirection) {
                        breakawayUp = volts;
                    } else {
                        breakawayDown = volts;
                    }
                    System.out.printf("[arm] broke away %s at %.3f V%n", label, volts);
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
                    velocityFit.add(volts, rps);
                    System.out.printf("[arm] %.2f V -> %.2f rot/s%n", volts, rps);
                    intake.stopDeploy();
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
        return Commands.sequence(
                Commands.runOnce(() -> {
                    double travel = intake.getDeployStops().getMeasuredTravel();
                    double low = intake.getDeployStops().getLearnedStop(End.LOW);
                    if (Double.isNaN(travel)) {
                        System.out.println("[arm] hold test skipped — travel not measured. "
                                + "Run the deploy travel calibration first.");
                        return;
                    }
                    intake.deployToGoal(low + travel * fraction);
                }, intake),
                Commands.waitSeconds(1.2),
                Commands.runOnce(() -> {
                    // Whatever the profiled controller is applying to hold station IS the holding
                    // voltage, so it can simply be read rather than searched for.
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
                    intake.setDeployVoltage(12.0 * LIMIT_TEST_OUTPUT);

                    double now = Timer.getFPGATimestamp();
                    double rps = Math.abs(intake.getDeployVelocity()) / 60.0;
                    double dt = now - lastVelocityTime;

                    if (dt > 1e-3 && dt < 0.1) {
                        peakAcceleration = Math.max(peakAcceleration,
                                Math.abs(rps - lastVelocity) / dt);
                    }
                    peakVelocity = Math.max(peakVelocity, rps);
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
