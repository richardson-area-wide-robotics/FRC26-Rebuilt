package frc.robot.common.components.diagnostics;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.CommonConstants.ModuleConstants;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * WPILib SysId, wired up, made automatic, and sized for half a field of carpet.
 *
 * <p>The documented SysId workflow is four separate button presses, then pulling a log onto a laptop
 * and reading gains off the analyser GUI. This keeps the standard routine — so the log is still a
 * normal SysId log and the desktop analyser still works on it if the plots are wanted — but adds the
 * three things that make it usable in a short shop session on a short field:
 *
 * <ul>
 *   <li><b>{@link #full()} chains every test</b> with pauses and console prompts, so it is one
 *       schedule rather than several presses in the right order with the right rests.
 *   <li><b>The fit happens on the robot.</b> The analyser's job is an ordinary least squares fit of
 *       {@code V = kS*sgn(v) + kV*v + kA*a}, which {@link SysIdRegression} does here. Gains print to
 *       the console when the run ends. No laptop step, no log transfer, no GUI.
 *   <li><b>It fits in 28 ft.</b> See below, because this is where the standard configuration falls
 *       over and the fix is not obvious.
 * </ul>
 *
 * <h2>Why bother, given the auto-calibrator already fits a feedforward</h2>
 *
 * <p>{@link DriveAutoCalibrator} sweeps duty cycle and waits for steady state, which fits kS and kV
 * well and <b>cannot fit kA at all</b> — at steady state acceleration is zero, so the data contains
 * no information about it. SysId's dynamic test is a voltage step, which is nothing but acceleration.
 * kA is what second-order kinematics needs, since that turns a chassis acceleration into module
 * accelerations and then needs something to turn those into volts.
 *
 * <p>They are complementary, and running both is a real cross-check rather than duplicated work: two
 * different excitations and two different regressions agreeing on kS and kV is evidence the numbers
 * are right. Disagreeing means one run was bad, which is far better learned from two printed numbers
 * than from a robot that follows paths oddly.
 *
 * <h2>Fitting into 28 ft of carpet</h2>
 *
 * <p>Half a field is <b>8.53 m</b>. A textbook SysId quasistatic ramp of 1 V/s for 6 s covers
 * <b>8.10 m</b> on this drivetrain — the entire carpet, before allowing for the robot's own length or
 * any stopping distance. As configured out of the box this test hits the wall.
 *
 * <p>The way out falls out of the arithmetic. Distance under a ramp is
 * {@code (rate * t^2 / 2 - kS * t) / kV}, and the final voltage is {@code rate * t}. Substituting,
 * <b>for a given final voltage the distance is inversely proportional to the ramp rate.</b> Reaching
 * 6 V at 1 V/s costs 8.10 m; reaching the same 6 V at 2 V/s costs 4.05 m. Half the runway, identical
 * voltage range, identical information about kV.
 *
 * <p>Normally you cannot spend that, because a fast ramp is no longer <em>quasi</em>static: it
 * carries real acceleration, which contaminates kS and kV in SysId's classical two-stage analysis
 * where the ramp is assumed to have none. <b>The combined fit here solves for kA at the same time, so
 * acceleration during the ramp is signal rather than contamination.</b> Doing the regression on the
 * robot is what buys the space back.
 *
 * <p>The dynamic test gets the same treatment differently. One long step spends most of its distance
 * travelling at steady state, which says nothing about kA — all the information is in the first few
 * time constants, and the time constant here is about 0.16 s. So instead of one long step this runs
 * <b>{@value #DYNAMIC_STEPS} short steps in alternating directions</b>: each one re-excites the
 * transient, and because they alternate the robot ends up roughly where it started. A tight field
 * yields <em>more</em> kA data this way, not less.
 *
 * <p>Planned excursions, on nominal constants: quasistatic about <b>4.1 m</b>, each dynamic step
 * about <b>0.9 m</b>. Both well inside the carpet.
 *
 * <p><b>Start at one end of the carpet, facing down its length.</b> Forward tests run out, reverse
 * tests come back, so the runway is needed once rather than per test.
 *
 * <h2>The abort is what actually keeps it off the wall</h2>
 *
 * <p>Those distance figures assume nominal kS and kV — and kV is the thing being measured, so the
 * prediction is circular and could be wrong in either direction. Every test therefore also stops
 * itself the moment the robot has travelled {@value #MAX_RUN_METERS} m from where that test started,
 * whatever the clock says. A test cut short by distance still contributes its samples; the report
 * says which tests were cut and the R² says whether what remains was enough.
 */
public class DriveSysId {

    /**
     * Volts per second for the quasistatic ramp.
     *
     * <p>1.5 rather than SysId's typical 1.0. See the class docs: for a given final voltage, distance
     * scales as 1/rate, and the combined fit tolerates the acceleration a faster ramp introduces.
     */
    private static final double RAMP_VOLTS_PER_SECOND = 1.5;

    /**
     * Seconds the quasistatic ramp runs.
     *
     * <p>3.5 s at 1.5 V/s reaches 5.25 V over about 4.1 m — half the carpet, with enough voltage
     * range for a solid linear fit.
     */
    private static final double RAMP_SECONDS = 3.5;

    /**
     * Step voltage for the dynamic tests.
     *
     * <p>3.5 V rather than SysId's default 7. A step exists to excite acceleration, and the size of
     * the step does not change the time constant — it only changes how far the robot travels while
     * the transient plays out. On a short field that is all cost and no benefit.
     */
    private static final double STEP_VOLTS = 3.5;

    /**
     * Seconds each dynamic step runs.
     *
     * <p>0.75 s is roughly 4.5 time constants, so the transient is fully captured, and about 0.9 m of
     * travel. Longer would only add steady-state samples, which carry no information about kA.
     */
    private static final double STEP_SECONDS = 0.75;

    /**
     * Dynamic steps, alternating direction.
     *
     * <p>Four short alternating steps capture four acceleration transients while ending roughly where
     * they began. One long step captures one transient and then drives for metres. Since the
     * transient is the only part that informs kA, this is strictly better on a short field.
     */
    private static final int DYNAMIC_STEPS = 4;

    /**
     * Metres from a test's own start at which it aborts, whatever the clock says.
     *
     * <p>6.0 m against 8.53 m of carpet leaves room for the robot's length and for it to stop. This
     * is the guard that does not depend on the distance predictions being right — and they cannot be
     * fully trusted, since they are computed from the kV this routine exists to measure.
     */
    public static final double MAX_RUN_METERS = 6.0;

    /** Half a field of carpet: 28 ft, which is the runway this team actually has. */
    public static final double HALF_FIELD_METERS = 28 * 0.3048;

    /**
     * Metres reserved for the robot's own length plus stopping distance.
     *
     * <p>The robot is a 26.5 in frame plus bumpers, which it is always tested with, so about 33 in or
     * 0.84 m of footprint — and it needs somewhere to stop after a run ends. 2.0 m of the carpet is
     * therefore not runway.
     */
    public static final double RESERVED_METERS = 2.0;

    /** Seconds of rest between tests, to let the drivetrain and the operator settle. */
    private static final double REST_SECONDS = 2.5;

    private static final String[] MODULE_NAMES = {"FL", "FR", "RL", "RR"};

    private final SwerveDriveSubsystem drive;

    /**
     * Separate routines per test type.
     *
     * <p>{@code SysIdRoutine.Config} carries one timeout for both quasistatic and dynamic, and these
     * need very different ones — 3.5 s of ramp against 0.75 s of step. One routine would force the
     * step to run as long as the ramp, which is exactly the metres this class is trying not to spend.
     */
    private final SysIdRoutine quasistaticRoutine;
    private final SysIdRoutine dynamicRoutine;

    /** One accumulator per module, because motors vary and an average hides a weak corner. */
    private final SysIdRegression.Accumulator[] perModule = {
        new SysIdRegression.Accumulator(),
        new SysIdRegression.Accumulator(),
        new SysIdRegression.Accumulator(),
        new SysIdRegression.Accumulator()
    };

    /** Previous velocities and timestamp, for the finite difference that gives acceleration. */
    private double[] previousVelocities;
    private double previousTimestamp;

    /** Where the current test started, for the distance abort. */
    private double runStartDistance;

    /** Longest excursion any single test needed, and how many were cut short. */
    private double worstRunMeters;
    private int abortedRuns;

    /**
     * @param drive The drivetrain.
     */
    public DriveSysId(SwerveDriveSubsystem drive) {
        this.drive = drive;

        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                voltage -> drive.driveVoltage(voltage.in(Volts)),
                log -> {
                    // The standard SysId log, so the desktop analyser still works on this if the
                    // on-robot fit ever looks suspect and the residual plots are wanted.
                    double[] positions = drive.getModuleDrivePositions();
                    double[] velocities = drive.getModuleDriveVelocities();
                    double[] volts = drive.getModuleDriveVoltages();

                    for (int i = 0; i < MODULE_NAMES.length; i++) {
                        log.motor("drive-" + MODULE_NAMES[i])
                                .voltage(Volts.of(volts[i]))
                                .linearPosition(Meters.of(positions[i]))
                                .linearVelocity(MetersPerSecond.of(velocities[i]));
                    }
                },
                drive,
                "drive");

        this.quasistaticRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(RAMP_VOLTS_PER_SECOND).per(Second),
                        Volts.of(STEP_VOLTS),
                        Seconds.of(RAMP_SECONDS),
                        state -> Logger.recordOutput("SysId/State", state.toString())),
                mechanism);

        this.dynamicRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(RAMP_VOLTS_PER_SECOND).per(Second),
                        Volts.of(STEP_VOLTS),
                        Seconds.of(STEP_SECONDS),
                        state -> Logger.recordOutput("SysId/State", state.toString())),
                mechanism);
    }

    /**
     * @return nominal volts per metre per second, from the drivetrain constants.
     *
     *     <p>The theoretical kV, used only to predict how far a test will travel. The measured kV is
     *     what this class exists to produce, so the prediction is necessarily circular — which is why
     *     {@link #MAX_RUN_METERS} exists as a guard that does not depend on it.
     */
    static double nominalKv() {
        return 12.0 / ModuleConstants.kDriveWheelFreeSpeedRps;
    }

    /**
     * @return predicted metres covered by one quasistatic ramp, on nominal constants.
     *
     *     <p>Integrating the quasi-steady velocity {@code (rate * t - kS) / kV} over the ramp. kS is
     *     taken as a nominal 0.18 V; it contributes only a small negative offset, so being wrong
     *     about it makes this prediction slightly conservative rather than dangerous.
     */
    static double plannedRampMeters() {
        double nominalKs = 0.18;
        return (RAMP_VOLTS_PER_SECOND * RAMP_SECONDS * RAMP_SECONDS / 2 - nominalKs * RAMP_SECONDS)
                / nominalKv();
    }

    /** @return voltage the quasistatic ramp reaches. */
    static double rampFinalVolts() {
        return RAMP_VOLTS_PER_SECOND * RAMP_SECONDS;
    }

    /**
     * Folds one loop's readings into the per-module fits.
     *
     * <p>Acceleration comes from a finite difference of velocity, which is what the desktop analyser
     * does too. It is noisy, and that is tolerable because least squares over hundreds of samples
     * averages zero-mean noise away — but it is also why a single bad sample cannot be spotted from
     * the printed gains alone, and why a poor R² means go and look at the log.
     */
    public void update() {
        double now = Timer.getFPGATimestamp();
        double[] velocities = drive.getModuleDriveVelocities();
        double[] volts = drive.getModuleDriveVoltages();

        if (previousVelocities != null) {
            double dt = now - previousTimestamp;

            // Guard both ends. A zero dt divides by nothing; a long one means the robot was disabled
            // in between, so the difference spans a gap and the implied acceleration is fiction.
            if (dt > 1e-3 && dt < 0.1) {
                for (int i = 0; i < velocities.length; i++) {
                    double acceleration = (velocities[i] - previousVelocities[i]) / dt;
                    perModule[i].add(volts[i], velocities[i], acceleration);
                }
            }
        }

        previousVelocities = velocities;
        previousTimestamp = now;
    }

    /** @return metres travelled since the current test began. */
    private double metersThisRun() {
        return Math.abs(drive.getAverageDriveDistance() - runStartDistance);
    }

    /**
     * Wraps a SysId command so samples are collected, distance is bounded, and the drivetrain stops.
     *
     * @param inner The SysId command.
     * @param label Console label.
     * @return the instrumented, distance-bounded command.
     */
    private Command instrumented(Command inner, String label) {
        return Commands.sequence(
                        Commands.runOnce(() -> {
                            previousVelocities = null;
                            runStartDistance = drive.getAverageDriveDistance();
                            System.out.println("[sysid] " + label + " starting");
                        }),
                        inner.alongWith(Commands.run(this::update))
                                // Whichever ends first. The clock is the plan; the distance is the
                                // wall.
                                .until(() -> metersThisRun() >= MAX_RUN_METERS),
                        Commands.runOnce(() -> {
                            drive.driveVoltage(0);

                            double used = metersThisRun();
                            worstRunMeters = Math.max(worstRunMeters, used);

                            if (used >= MAX_RUN_METERS) {
                                abortedRuns++;
                                System.out.printf("[sysid] %s CUT SHORT at %.2f m (limit %.1f m) "
                                                + "— samples still counted%n",
                                        label, used, MAX_RUN_METERS);
                            } else {
                                System.out.printf("[sysid] %s done, used %.2f m%n", label, used);
                            }
                        }, drive))
                .withName("SysId/" + label);
    }

    /** @return the quasistatic ramp, forward. Informs kS and kV. */
    public Command quasistaticForward() {
        return instrumented(quasistaticRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                "quasistatic forward");
    }

    /** @return the quasistatic ramp, reverse. */
    public Command quasistaticReverse() {
        return instrumented(quasistaticRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
                "quasistatic reverse");
    }

    /** @return one dynamic step, forward. This is the test that makes kA measurable. */
    public Command dynamicForward() {
        return instrumented(dynamicRoutine.dynamic(SysIdRoutine.Direction.kForward),
                "dynamic forward");
    }

    /** @return one dynamic step, reverse. */
    public Command dynamicReverse() {
        return instrumented(dynamicRoutine.dynamic(SysIdRoutine.Direction.kReverse),
                "dynamic reverse");
    }

    /**
     * All tests in sequence, then the report. Sized for 28 ft of carpet.
     *
     * <p>Quasistatic first: it is the gentler test, so a mechanical problem shows up before the
     * drivetrain is asked to take a voltage step.
     *
     * <p><b>Both directions matter and are not redundant.</b> Running only forward leaves kS
     * confounded with any directional asymmetry in the drivetrain — a dragging brake or a tight
     * bearing shows up as a larger kS rather than as the mechanical fault it is. Alternating also
     * returns the robot to roughly where it started, which is what makes the whole sequence fit on
     * half a field.
     *
     * @return the full characterisation.
     */
    public Command full() {
        Command[] phases = new Command[4 + DYNAMIC_STEPS * 2];
        int at = 0;

        phases[at++] = Commands.runOnce(() -> {
            reset();
            System.out.println("[sysid] === Drive characterisation ===");
            System.out.printf("[sysid] Sized for half a field. Start at one end facing down the "
                            + "carpet. Longest single run is about 4.1 m; hard abort at %.1f m.%n",
                    MAX_RUN_METERS);
        });

        phases[at++] = quasistaticForward();
        phases[at++] = Commands.waitSeconds(REST_SECONDS);
        phases[at++] = quasistaticReverse();

        // Alternating short steps: each re-excites the transient that carries the kA information,
        // and the alternation keeps the robot near where it started.
        for (int step = 0; step < DYNAMIC_STEPS; step++) {
            phases[at++] = Commands.waitSeconds(REST_SECONDS);
            phases[at++] = step % 2 == 0 ? dynamicForward() : dynamicReverse();
        }

        return Commands.sequence(phases)
                .andThen(Commands.runOnce(this::printReport))
                .withName("SysId/Full");
    }

    /** Mean gains across the four modules, with the spread that says whether the mean is honest. */
    public record Summary(double kS, double kV, double kA, double kaSpreadPercent,
            String worstModule, int trustworthyModules) {

        /** @return true when all four modules produced a usable fit. */
        public boolean isComplete() {
            return trustworthyModules == 4;
        }

        /**
         * @return true when these gains may be written into constants.
         *
         *     <p>Deliberately stricter than {@link #isComplete()}. A mean built from three good
         *     modules is a legitimate <em>estimate</em> and worth printing, but it is not a number to
         *     bake in: whatever stopped the fourth module fitting has not been explained, and a
         *     feedforward that under-drives one corner reads as the robot yawing under acceleration
         *     rather than as a gains problem.
         */
        public boolean isSafeToAdopt() {
            return isComplete() && kV > 0 && kS >= 0 && kA >= 0 && kaSpreadPercent <= 25.0;
        }
    }

    /**
     * Averages the four module fits.
     *
     * <p>The mean is what a chassis-level feedforward wants — second-order kinematics turns a chassis
     * acceleration into module accelerations, and kA converts those into volts.
     *
     * <p><b>The spread is reported because the mean alone can hide the thing you care about.</b> Four
     * modules within a few percent means the mean describes all of them. One corner 30% off means
     * that corner accelerates differently, and a chassis feedforward on the average will under-drive
     * it — which shows up as the robot yawing under hard acceleration rather than as anything
     * obviously feedforward-related.
     *
     * @return the summary.
     */
    public Summary summarise() {
        double sumS = 0;
        double sumV = 0;
        double sumA = 0;
        double minA = Double.POSITIVE_INFINITY;
        double maxA = Double.NEGATIVE_INFINITY;
        String worst = "none";
        int trustworthy = 0;

        // Only trustworthy fits contribute. This used to sum all four unconditionally, and a
        // module whose fit failed contributes (0, 0, 0) rather than nothing -- so one unplugged drive
        // encoder pulled the mean to 75% of the truth, and a kV a quarter low under-drives every path
        // of the season while reading as a tuning problem.
        //
        // The spread has to be computed over the same population for the same reason: a zero from a
        // failed fit is the minimum kA by a distance, so the spread was near 100% and the warning
        // blamed a mechanical fault on whichever module merely had the highest kA.
        for (int i = 0; i < MODULE_NAMES.length; i++) {
            SysIdRegression.Gains gains = perModule[i].fit();
            if (!gains.isTrustworthy()) {
                continue;
            }
            trustworthy++;

            sumS += gains.kS();
            sumV += gains.kV();
            sumA += gains.kA();

            if (gains.kA() < minA) {
                minA = gains.kA();
            }
            if (gains.kA() > maxA) {
                maxA = gains.kA();
                worst = MODULE_NAMES[i];
            }
        }

        if (trustworthy == 0) {
            return new Summary(0, 0, 0, 0, "none", 0);
        }

        double meanA = sumA / trustworthy;
        double spread = meanA > 1e-9 ? (maxA - minA) / meanA * 100.0 : 0;

        return new Summary(sumS / trustworthy, sumV / trustworthy, meanA, spread, worst,
                trustworthy);
    }

    /** Prints the per-module fits and the mean. */
    public void printReport() {
        System.out.println();
        System.out.println("=== SYSID FEEDFORWARD REPORT ===");
        System.out.println("Fitted on-robot: V = kS*sgn(v) + kV*v + kA*a");
        System.out.println();

        for (int i = 0; i < MODULE_NAMES.length; i++) {
            SysIdRegression.Gains gains = perModule[i].fit();
            System.out.println("  " + MODULE_NAMES[i] + ": " + gains.describe());

            String root = "SysId/" + MODULE_NAMES[i];
            Logger.recordOutput(root + "/kS", gains.kS());
            Logger.recordOutput(root + "/kV", gains.kV());
            Logger.recordOutput(root + "/kA", gains.kA());
            Logger.recordOutput(root + "/RSquared", gains.rSquared());
            Logger.recordOutput(root + "/Samples", gains.samples());
            Logger.recordOutput(root + "/Trustworthy", gains.isTrustworthy());
        }

        Summary summary = summarise();

        // Published for tools/apply_sysid.py, which writes these into the constants so nobody has
        // to retype four-decimal numbers off a console. The stamp is what makes that safe: it lets the
        // script tell a fresh result from a value still sitting in NetworkTables from an earlier run,
        // which is the one way an automatic paste is worse than a manual one.
        String root = "SysId/Summary";
        Logger.recordOutput(root + "/kS", summary.kS());
        Logger.recordOutput(root + "/kV", summary.kV());
        Logger.recordOutput(root + "/kA", summary.kA());
        Logger.recordOutput(root + "/SpreadPercent", summary.kaSpreadPercent());
        Logger.recordOutput(root + "/TrustworthyModules", summary.trustworthyModules());
        Logger.recordOutput(root + "/Complete", summary.isComplete());
        Logger.recordOutput(root + "/SafeToAdopt", summary.isSafeToAdopt());
        Logger.recordOutput(root + "/WorstModule", summary.worstModule());
        Logger.recordOutput(root + "/Stamp", Timer.getFPGATimestamp());

        System.out.println();
        System.out.printf("  MEAN of %d trustworthy module(s): kS = %.4f  kV = %.4f  kA = %.4f%n",
                summary.trustworthyModules(), summary.kS(), summary.kV(), summary.kA());
        System.out.printf("  kA spread across those modules: %.1f%% (highest at %s)%n",
                summary.kaSpreadPercent(), summary.worstModule());

        // The paste line is withheld rather than printed with a caveat under it. It used to be
        // printed first and unconditionally, so the caveat arrived after the thing it was qualifying
        // and blocked nothing -- and a paste line reads as permission.
        if (summary.isSafeToAdopt()) {
            System.out.printf("  Paste into CommonConstants.DriveFeedforwardConstants: "
                            + "kS = %.4f; kV = %.4f; kA = %.4f;%n",
                    summary.kS(), summary.kV(), summary.kA());
            System.out.println("  Or let the script do it:  python tools/apply_sysid.py --yes");
        } else {
            System.out.println("  NO VALUES TO ADOPT YET. Nothing above should be pasted.");
        }

        if (summary.kaSpreadPercent() > 25) {
            System.out.println("  ^ over 25% — one corner accelerates differently from the others.");
            System.out.println("    A chassis feedforward on the mean will under-drive it, which");
            System.out.println("    reads as the robot yawing under hard acceleration rather than");
            System.out.println("    as a feedforward problem. Check that corner mechanically.");
        }

        if (!summary.isComplete()) {
            System.out.printf("  ^ only %d of 4 modules produced a trustworthy fit — the mean is "
                    + "not reliable yet.%n", summary.trustworthyModules());
        }

        System.out.println();
        System.out.printf("  Space used: longest single run %.2f m against a %.1f m limit; "
                        + "%d run(s) cut short.%n",
                worstRunMeters, MAX_RUN_METERS, abortedRuns);

        if (abortedRuns > 0) {
            System.out.println("  ^ runs hit the distance limit, so they carry less voltage range");
            System.out.println("    than planned. If R2 is good the fit is still fine. If not,");
            System.out.println("    raise RAMP_VOLTS_PER_SECOND — a faster ramp reaches the same");
            System.out.println("    voltage in less distance, and the combined fit tolerates the");
            System.out.println("    extra acceleration.");
        }

        Logger.recordOutput("SysId/Mean/kS", summary.kS());
        Logger.recordOutput("SysId/Mean/kV", summary.kV());
        Logger.recordOutput("SysId/Mean/kA", summary.kA());
        Logger.recordOutput("SysId/Mean/KaSpreadPercent", summary.kaSpreadPercent());
        Logger.recordOutput("SysId/Mean/TrustworthyModules", summary.trustworthyModules());
        Logger.recordOutput("SysId/WorstRunMeters", worstRunMeters);
        Logger.recordOutput("SysId/AbortedRuns", abortedRuns);

        System.out.println();
        System.out.println("kA is what second-order kinematics needs: it converts a commanded");
        System.out.println("module acceleration into volts. It is only available here — the");
        System.out.println("auto-calibrator's sweep waits for steady state, so acceleration is");
        System.out.println("zero by construction and kA is unmeasurable from that data.");
        System.out.println();
        System.out.println("Cross-check kS and kV against the auto-calibrator's figures. Two");
        System.out.println("different excitations and two different regressions agreeing is real");
        System.out.println("evidence; disagreeing means one of the runs was bad.");
        System.out.println("=== END ===");
        System.out.println();
    }

    /** Clears every accumulated sample. */
    public void reset() {
        for (SysIdRegression.Accumulator accumulator : perModule) {
            accumulator.reset();
        }
        previousVelocities = null;
        worstRunMeters = 0;
        abortedRuns = 0;
    }

    /**
     * @param module Index in FL, FR, RL, RR order.
     * @return that module's fit.
     */
    public SysIdRegression.Gains getGains(int module) {
        return perModule[module].fit();
    }

    /** @return metres of runway the longest single test needed. */
    public double getWorstRunMeters() {
        return worstRunMeters;
    }

    /** @return how many tests were cut short by the distance limit. */
    public int getAbortedRuns() {
        return abortedRuns;
    }
}
