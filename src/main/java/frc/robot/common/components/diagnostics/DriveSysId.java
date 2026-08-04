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
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * WPILib SysId, wired up and made automatic.
 *
 * <p>The documented SysId workflow is four separate button presses, then pulling a log onto a laptop
 * and reading gains off the analyser GUI. This keeps the standard routine — so the log is still a
 * normal SysId log and the desktop analyser still works on it if you want the plots — but adds the
 * two things that make it usable in a short shop session:
 *
 * <ul>
 *   <li><b>{@link #full()} chains all four tests</b> with pauses and console prompts between them, so
 *       it is one schedule rather than four presses in the right order with the right rests.
 *   <li><b>The fit happens on the robot.</b> The analyser's job is an ordinary least squares fit of
 *       {@code V = kS*sgn(v) + kV*v + kA*a}, which {@link SysIdRegression} does here. Gains print to
 *       the console when the run ends. No laptop step, no log transfer, no GUI.
 * </ul>
 *
 * <h2>Why bother, given the auto-calibrator already fits a feedforward</h2>
 *
 * <p>{@link DriveAutoCalibrator} sweeps duty cycle and waits for steady state, which fits kS and kV
 * well and <b>cannot fit kA at all</b> — at steady state acceleration is zero, so the data contains
 * no information about it. SysId's dynamic test is a voltage step, which is nothing but acceleration.
 *
 * <p>So they are complementary, and running both is a real cross-check rather than duplicated work:
 * two different excitations and two different regressions agreeing on kS and kV is evidence the
 * numbers are right. Disagreeing means one run was bad, which is far better learned from two printed
 * numbers than from a robot that follows paths oddly.
 *
 * <h2>Space</h2>
 *
 * <p><b>This needs more room than anything else in the calibration suite — plan on 10 m of clear
 * runway.</b> That is the honest cost of SysId on a drivetrain: the quasistatic ramp has to reach a
 * useful voltage, and the robot keeps accelerating the whole time it does. The forward and reverse
 * tests return the robot roughly to where it started, so the runway is needed once, not per test.
 *
 * <p>Each test stops itself on a timeout. If space is tight, lower {@link #RAMP_SECONDS} — the fit
 * degrades gracefully because it simply sees a smaller voltage range, and the reported R² will say
 * whether what remains was enough.
 */
public class DriveSysId {

    /** Volts per second for the quasistatic ramp. */
    private static final double RAMP_VOLTS_PER_SECOND = 1.0;

    /**
     * Seconds the quasistatic ramp runs.
     *
     * <p>6 s at 1 V/s reaches 6 V, which is half the bus and plenty of range for a linear fit.
     * Distance covered is roughly 8 m, which is what sets the runway requirement.
     */
    private static final double RAMP_SECONDS = 6.0;

    /**
     * Step voltage for the dynamic test.
     *
     * <p>4 V rather than SysId's default 7. The dynamic test only needs to capture the acceleration
     * transient, which is over in a few hundred milliseconds, and 7 V would spend the rest of the
     * window travelling fast for no extra information.
     */
    private static final double STEP_VOLTS = 4.0;

    /** Seconds the dynamic step runs. Long enough for the transient plus a little steady state. */
    private static final double STEP_SECONDS = 2.0;

    /** Seconds of rest between tests, to let the drivetrain and the operator settle. */
    private static final double REST_SECONDS = 3.0;

    private static final String[] MODULE_NAMES = {"FL", "FR", "RL", "RR"};

    private final SwerveDriveSubsystem drive;
    private final SysIdRoutine routine;

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

    /** Voltage most recently commanded, needed to pair with each sample. */
    private double commandedVolts;

    /**
     * @param drive The drivetrain.
     */
    public DriveSysId(SwerveDriveSubsystem drive) {
        this.drive = drive;

        this.routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(RAMP_VOLTS_PER_SECOND).per(Second),
                        Volts.of(STEP_VOLTS),
                        Seconds.of(Math.max(RAMP_SECONDS, STEP_SECONDS)),
                        state -> Logger.recordOutput("SysId/State", state.toString())),
                new SysIdRoutine.Mechanism(
                        voltage -> {
                            commandedVolts = voltage.in(Volts);
                            drive.driveVoltage(commandedVolts);
                        },
                        log -> {
                            // The standard SysId log, so the desktop analyser still works on this
                            // if the on-robot fit ever looks suspect and the plots are wanted.
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
                        "drive"));
    }

    /**
     * Folds one loop's readings into the per-module fits.
     *
     * <p>Acceleration comes from a finite difference of velocity, which is what the desktop analyser
     * does too. It is noisy, and that is tolerable here because least squares over hundreds of
     * samples averages zero-mean noise away — but it is also why a single bad sample cannot be
     * spotted from the printed gains alone, and why a poor R² means go and look at the log.
     *
     * <p>Call this every loop while a test is running. Between tests it is harmless: samples below
     * the velocity threshold are discarded.
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

    /** @return the quasistatic ramp, forward. Informs kS and kV. */
    public Command quasistaticForward() {
        return instrumented(routine.quasistatic(SysIdRoutine.Direction.kForward),
                "quasistatic forward");
    }

    /** @return the quasistatic ramp, reverse. */
    public Command quasistaticReverse() {
        return instrumented(routine.quasistatic(SysIdRoutine.Direction.kReverse),
                "quasistatic reverse");
    }

    /** @return the dynamic step, forward. This is the test that makes kA measurable. */
    public Command dynamicForward() {
        return instrumented(routine.dynamic(SysIdRoutine.Direction.kForward), "dynamic forward");
    }

    /** @return the dynamic step, reverse. */
    public Command dynamicReverse() {
        return instrumented(routine.dynamic(SysIdRoutine.Direction.kReverse), "dynamic reverse");
    }

    /**
     * All four tests in sequence, then the report.
     *
     * <p>This is the automation the documented workflow lacks: one schedule instead of four button
     * presses in the correct order with the correct rests, and no log transfer afterwards.
     *
     * <p><b>Both directions matter and are not redundant.</b> Running only forward leaves kS
     * confounded with any directional asymmetry in the drivetrain — a dragging brake or a
     * tight bearing on one side shows up as a larger kS rather than as the mechanical fault it is.
     * Forward and reverse together also return the robot roughly to its starting point, so the
     * runway is needed once rather than four times.
     *
     * <p>Order is quasistatic first: it is the gentler test, so if something is wrong mechanically it
     * shows up before the drivetrain is asked to take a 4 V step.
     *
     * @return the full characterisation.
     */
    public Command full() {
        return Commands.sequence(
                        Commands.runOnce(() -> {
                            reset();
                            System.out.println("[sysid] === Drive characterisation ===");
                            System.out.println("[sysid] Needs about 10 m of clear runway. "
                                    + "Four tests, roughly 30 s including rests.");
                        }),
                        quasistaticForward(),
                        Commands.waitSeconds(REST_SECONDS),
                        quasistaticReverse(),
                        Commands.waitSeconds(REST_SECONDS),
                        dynamicForward(),
                        Commands.waitSeconds(REST_SECONDS),
                        dynamicReverse(),
                        Commands.runOnce(this::printReport))
                .withName("SysId/Full");
    }

    /**
     * Wraps a SysId command so samples are collected while it runs and the drivetrain is stopped
     * afterwards.
     *
     * @param inner The SysId command.
     * @param label Console label.
     * @return the instrumented command.
     */
    private Command instrumented(Command inner, String label) {
        return Commands.sequence(
                        Commands.runOnce(() -> {
                            previousVelocities = null;
                            System.out.println("[sysid] " + label + " starting");
                        }),
                        inner.alongWith(Commands.run(this::update)),
                        Commands.runOnce(() -> drive.driveVoltage(0), drive))
                .withName("SysId/" + label);
    }

    /** Mean gains across the four modules, with the spread that says whether the mean is honest. */
    public record Summary(double kS, double kV, double kA, double kaSpreadPercent,
            String worstModule, int trustworthyModules) {

        /** @return true when all four modules produced a usable fit. */
        public boolean isComplete() {
            return trustworthyModules == 4;
        }
    }

    /**
     * Averages the four module fits.
     *
     * <p>The mean is what a chassis-level feedforward wants — second-order kinematics turns a chassis
     * acceleration into a module acceleration, and kA converts that into volts, so it needs one
     * number per module or one for the drivetrain.
     *
     * <p><b>The spread is reported because the mean alone can hide the thing you care about.</b> Four
     * modules with kA within a few percent means the mean describes all of them. One corner 30% off
     * means that corner accelerates differently from the other three, and a chassis feedforward built
     * on the average will under-drive it — which shows up as the robot yawing under hard acceleration
     * rather than as anything obviously feedforward-related.
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

        for (int i = 0; i < MODULE_NAMES.length; i++) {
            SysIdRegression.Gains gains = perModule[i].fit();
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

            if (gains.isTrustworthy()) {
                trustworthy++;
            }
        }

        int n = MODULE_NAMES.length;
        double meanA = sumA / n;
        double spread = meanA > 1e-9 ? (maxA - minA) / meanA * 100.0 : 0;

        return new Summary(sumS / n, sumV / n, meanA, spread, worst, trustworthy);
    }

    /** Prints the per-module fits. */
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

        System.out.println();
        System.out.printf("  MEAN: kS = %.4f  kV = %.4f  kA = %.4f%n",
                summary.kS(), summary.kV(), summary.kA());
        System.out.printf("  Paste into CommonConstants.DriveFeedforwardConstants: "
                        + "kS = %.4f; kV = %.4f; kA = %.4f;%n",
                summary.kS(), summary.kV(), summary.kA());
        System.out.printf("  kA spread across modules: %.1f%% (highest at %s)%n",
                summary.kaSpreadPercent(), summary.worstModule());

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

        Logger.recordOutput("SysId/Mean/kS", summary.kS());
        Logger.recordOutput("SysId/Mean/kV", summary.kV());
        Logger.recordOutput("SysId/Mean/kA", summary.kA());
        Logger.recordOutput("SysId/Mean/KaSpreadPercent", summary.kaSpreadPercent());
        Logger.recordOutput("SysId/Mean/TrustworthyModules", summary.trustworthyModules());

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
    }

    /** @return the per-module fit, in FL, FR, RL, RR order. */
    public SysIdRegression.Gains getGains(int module) {
        return perModule[module].fit();
    }
}
