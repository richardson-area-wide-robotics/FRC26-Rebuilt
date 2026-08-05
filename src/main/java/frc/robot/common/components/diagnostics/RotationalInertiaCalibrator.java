package frc.robot.common.components.diagnostics;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CommonConstants.ModuleConstants;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Measures the robot's rotational inertia by spinning it and watching how fast it speeds up.
 *
 * <p>{@code settings.json} carries {@code robotMOI = 3.733 kg·m²} and PathPlanner uses it to plan
 * rotational acceleration. It is the one number in that file that <b>cannot be measured with a tape
 * measure and normally comes from CAD</b> — and on this robot CAD cannot supply it either, because the
 * assembly is too large for the mass-properties tool to finish.
 *
 * <p>So measure it from the robot's own dynamics instead, which has the advantage of including the
 * wire, tape and zip ties that a CAD model never has. Newton's second law for rotation:
 *
 * <pre>
 *   I = torque / angular acceleration
 * </pre>
 *
 * <p>Spin the chassis in place at a fixed voltage, take the torque from the measured motor current
 * and the geometry, and take the angular acceleration from the gyro. Both sides are things this robot
 * already reports every loop.
 *
 * <h2>Why the intake position gets its own measurement</h2>
 *
 * <p>Deploying the intake moves several kilograms from tucked against the frame to well outside it,
 * and rotational inertia goes as <b>mass times radius squared</b> — so it is not a small effect.
 * Moving 5 kg from 0.20 m to 0.45 m adds 0.81 kg·m², which is <b>22%</b> of the figure currently in
 * the file.
 *
 * <p>That means <b>a single {@code robotMOI} is necessarily wrong for one of the two states</b>, and
 * no amount of gain tuning fixes it — PathPlanner will plan rotational accelerations the robot cannot
 * achieve in one configuration or will under-drive it in the other. This measures both and reports the
 * difference so the choice is informed rather than accidental.
 *
 * <h2>What invalidates a run</h2>
 *
 * <p><b>Wheel slip.</b> The torque figure assumes every newton the motors make reaches the carpet. If
 * the wheels are slipping, the robot accelerates less than the current implies and the inertia comes
 * out too high. During a pure spin every wheel is tangential, so slip is detectable the same way as in
 * the traction test: a wheel's speed should be exactly the angular rate times the drive radius, and
 * anything faster is slipping. Checked per run and reported.
 *
 * <p>A modest voltage keeps well clear of it. At 15 A per motor the spin demands a coefficient of
 * friction of only 0.26, against a carpet that manages something like 1.0.
 */
public class RotationalInertiaCalibrator {

    /**
     * Voltage for the spin step.
     *
     * <p>2.5 V is enough to produce an angular acceleration the gyro measures easily while demanding
     * far less grip than the carpet has. Going harder would risk slip, which biases the answer high.
     */
    private static final double SPIN_VOLTS = 2.5;

    /**
     * Seconds of spinning.
     *
     * <p>Short. Inertia is read from the <b>initial</b> slope of angular rate, where aerodynamic and
     * bearing drag have barely begun to matter. A long spin reaches a terminal rate governed by drag
     * rather than inertia, which measures the wrong thing.
     */
    private static final double SPIN_SECONDS = 0.6;

    /** Seconds to discard at the start, while the modules rotate to their tangential angles. */
    private static final double SETTLE_SECONDS = 0.4;

    /** Seconds between runs, to let the robot stop and the operator reposition. */
    private static final double REST_SECONDS = 2.5;

    /**
     * Fraction by which wheel speed may exceed the gyro-implied speed before it counts as slip.
     *
     * <p>1.15 allows 15%. Some excess is expected from wheel-scale error and the gyro's own lag, and
     * neither is slip. A slipping wheel exceeds it by far more than that.
     */
    private static final double SLIP_RATIO = 1.15;

    /** NEO Vortex stall torque, stall current and free current, from REV's datasheet. */
    private static final double VORTEX_STALL_TORQUE_NM = 3.60;
    private static final double VORTEX_STALL_CURRENT_AMPS = 211.0;
    private static final double VORTEX_FREE_CURRENT_AMPS = 3.6;

    /**
     * Torque constant, in newton-metres per amp of <b>useful</b> current.
     *
     * <p>Note the denominator. This was stall torque over stall <em>current</em>, which is not the
     * torque constant: some of the current at any operating point produces no torque at all, and the
     * datasheet's free current is what that costs. So Kt is stall torque over
     * {@code stall - free} current, and the torque produced is {@code Kt * (I - I_free)} -- see
     * {@link #usefulTorqueNm}.
     *
     * <p>The error was large and in one direction. At the 15 A per motor this routine actually
     * operates at, the old arithmetic gave 0.256 N.m where the truth is 0.198 -- a <b>29%
     * overestimate of torque, and therefore of the inertia computed from it</b>, before any allowance
     * for gearbox efficiency, which pushes the same way. That figure was printed to three decimals and
     * pasted into PathPlanner's {@code robotMOI}, where it plans every rotation of the season.
     *
     * <p>The fraction lost to no-load current is large here precisely <em>because</em>
     * {@code SPIN_VOLTS} is deliberately small: at low current, a fixed 3.6 A of it is a big share.
     */
    public static final double TORQUE_PER_AMP =
            VORTEX_STALL_TORQUE_NM / (VORTEX_STALL_CURRENT_AMPS - VORTEX_FREE_CURRENT_AMPS);

    /**
     * @param amps Measured current for one motor.
     * @return the torque that current actually produces, in newton-metres.
     *
     *     <p>Clamped at zero: below the free current a motor produces no useful torque, and a negative
     *     torque here would come out as a negative inertia, which is not a measurement of anything.
     */
    public static double usefulTorqueNm(double amps) {
        return Math.max(0.0, (amps - VORTEX_FREE_CURRENT_AMPS) * TORQUE_PER_AMP);
    }

    /** Which configuration a run was taken in. */
    public enum IntakeState {
        /** Intake stowed against the frame. */
        STOWED,
        /** Intake deployed, mass further from the centre. */
        DEPLOYED
    }

    /** One measurement. */
    public record Result(IntakeState state, double momentOfInertia, double torqueNm,
            double angularAccelRadPerSec2, double perMotorAmps, double rSquared,
            double worstSlipRatio, int samples) {

        /** @return true when the run is worth believing. */
        public boolean isValid() {
            return samples >= 20
                    && rSquared >= 0.90
                    && worstSlipRatio <= SLIP_RATIO
                    && momentOfInertia > 0
                    && angularAccelRadPerSec2 > 0.5;
        }

        /** @return a readable line, with the reason if it is not usable. */
        public String describe() {
            if (worstSlipRatio > SLIP_RATIO) {
                return String.format(
                        "%s: INVALID — wheels ran %.0f%% faster than the spin rate implies, so they "
                                + "were slipping. Torque did not all reach the carpet and the "
                                + "inertia would come out too high. Lower SPIN_VOLTS and re-run.",
                        state, (worstSlipRatio - 1) * 100);
            }

            if (samples < 20 || angularAccelRadPerSec2 <= 0.5) {
                return String.format(
                        "%s: INVALID — the robot barely rotated (%.2f rad/s^2 over %d samples). "
                                + "Check it is on the floor with room to spin and that the modules "
                                + "reached their tangential angles.",
                        state, angularAccelRadPerSec2, samples);
            }

            String verdict = rSquared >= 0.90
                    ? "OK"
                    : String.format("SUSPECT: R2 %.3f — the rate did not rise linearly, so something "
                            + "other than a constant torque was acting", rSquared);

            return String.format(
                    "%s: I = %.3f kg.m^2   (%.1f N.m from %.1f A/motor, %.2f rad/s^2, R2 %.3f, "
                            + "%d samples) — %s",
                    state, momentOfInertia, torqueNm, perMotorAmps, angularAccelRadPerSec2,
                    rSquared, samples, verdict);
        }
    }

    /**
     * Least squares slope of angular rate against time.
     *
     * <p>Deliberately not {@link DriveCharacterization.FeedforwardFit}, which discards samples whose
     * velocity is near zero. That is right for a feedforward sweep and wrong here: the samples where
     * the rate is still small are the start of the rise, which is exactly the part that carries the
     * inertia.
     */
    static final class SlopeFit {
        private int n;
        private double sumX;
        private double sumY;
        private double sumXx;
        private double sumXy;
        private double sumYy;

        void add(double x, double y) {
            n++;
            sumX += x;
            sumY += y;
            sumXx += x * x;
            sumXy += x * y;
            sumYy += y * y;
        }

        int count() {
            return n;
        }

        /** @return the slope, or 0 if it cannot be determined. */
        double slope() {
            double denominator = n * sumXx - sumX * sumX;
            return Math.abs(denominator) < 1e-12 ? 0 : (n * sumXy - sumX * sumY) / denominator;
        }

        /** @return coefficient of determination, 0 when undefined. */
        double rSquared() {
            if (n < 3) {
                return 0;
            }
            double slope = slope();
            double intercept = (sumY - slope * sumX) / n;

            double ssTot = sumYy - sumY * sumY / n;
            double ssRes = sumYy
                    - 2 * (slope * sumXy + intercept * sumY)
                    + slope * slope * sumXx + 2 * slope * intercept * sumX + n * intercept * intercept;

            if (ssTot <= 1e-12) {
                return 0;
            }
            return Math.max(0, Math.min(1, 1 - ssRes / ssTot));
        }

        void reset() {
            n = 0;
            sumX = sumY = sumXx = sumXy = sumYy = 0;
        }
    }

    /**
     * Torque about the chassis centre, from the current each drive motor is drawing.
     *
     * <p>Four motors, each making {@link #usefulTorqueNm} at the shaft. Through the gearing that
     * becomes {@code * reduction} at the wheel, and dividing by the wheel radius gives the force at the
     * carpet. Each force acts at the drive radius, so the torques add.
     *
     * <p>Uses {@link #usefulTorqueNm} rather than a flat torque-per-amp, so the no-load current is
     * subtracted before the current is turned into torque. That correction is worth about 29% at this
     * routine's operating point, all in one direction -- see {@link #TORQUE_PER_AMP}.
     *
     * @param perMotorAmps Mean current per drive motor.
     * @return torque about the chassis centre, in newton-metres.
     */
    public static double torqueFromCurrent(double perMotorAmps) {
        double wheelForce = usefulTorqueNm(perMotorAmps) * ModuleConstants.kDrivingMotorReduction
                / (ModuleConstants.kWheelDiameterMeters / 2);
        return 4 * wheelForce * SwerveDriveSubsystem.getDriveRadiusMeters();
    }

    /**
     * @param perMotorAmps Mean current per drive motor.
     * @param angularAccel Measured angular acceleration, rad/s².
     * @return moment of inertia about the vertical axis, kg·m². Zero if acceleration is not positive.
     */
    public static double estimateMoi(double perMotorAmps, double angularAccel) {
        if (angularAccel <= 1e-6) {
            return 0;
        }
        return torqueFromCurrent(perMotorAmps) / angularAccel;
    }

    private final SwerveDriveSubsystem drive;
    private final Runnable stowIntake;
    private final Runnable deployIntake;
    private final BooleanSupplier hasIntake;

    private final SlopeFit rateFit = new SlopeFit();
    private final VisionCalibration.RunningStats currentStats = new VisionCalibration.RunningStats();

    private double runStartTime;
    private double worstSlipRatio;

    private Result stowedResult;
    private Result deployedResult;

    /**
     * @param drive        The drivetrain.
     * @param stowIntake   Stows the intake, or null if there is no intake to move.
     * @param deployIntake Deploys the intake, or null.
     */
    public RotationalInertiaCalibrator(SwerveDriveSubsystem drive, Runnable stowIntake,
            Runnable deployIntake) {
        this.drive = drive;
        this.stowIntake = stowIntake;
        this.deployIntake = deployIntake;
        this.hasIntake = () -> stowIntake != null && deployIntake != null;
    }

    /** Folds in one loop of the spin. */
    private void sample() {
        double now = Timer.getFPGATimestamp();
        double omega = Math.toRadians(drive.getTurnRate());
        double perMotorAmps = drive.getTotalDriveCurrent() / 4.0;

        rateFit.add(now - runStartTime, Math.abs(omega));
        currentStats.add(perMotorAmps);

        // Slip check. In a pure spin every wheel should run at exactly omega * driveRadius; faster
        // than that and it is turning without carrying the robot with it.
        double impliedWheelSpeed = Math.abs(omega) * SwerveDriveSubsystem.getDriveRadiusMeters();
        double actualWheelSpeed = drive.getAverageAbsoluteDriveVelocity();
        if (impliedWheelSpeed > 0.05) {
            worstSlipRatio = Math.max(worstSlipRatio, actualWheelSpeed / impliedWheelSpeed);
        }
    }

    /**
     * Builds one spin measurement.
     *
     * @param state Which configuration this run represents.
     * @return the command.
     */
    private Command spinRun(IntakeState state) {
        return Commands.sequence(
                        Commands.runOnce(() -> {
                            rateFit.reset();
                            currentStats.reset();
                            worstSlipRatio = 0;
                            System.out.println("[inertia] " + state + " spin starting");
                        }),
                        // Let the modules reach their tangential angles before anything is recorded.
                        // Sampling through the steering transient would fit a slope to a period when
                        // the wheels were not yet pushing the right way.
                        Commands.run(() -> drive.spinOpenLoop(SPIN_VOLTS), drive)
                                .withTimeout(SETTLE_SECONDS),
                        Commands.runOnce(() -> runStartTime = Timer.getFPGATimestamp()),
                        Commands.run(() -> {
                            drive.spinOpenLoop(SPIN_VOLTS);
                            sample();
                        }, drive).withTimeout(SPIN_SECONDS),
                        Commands.runOnce(() -> {
                            drive.spinOpenLoop(0);
                            record(state);
                        }, drive),
                        Commands.waitSeconds(REST_SECONDS))
                .withName("Inertia/" + state);
    }

    /** Turns the accumulated samples into a result. */
    private void record(IntakeState state) {
        double alpha = rateFit.slope();
        double amps = currentStats.getMean();

        Result result = new Result(state, estimateMoi(amps, alpha), torqueFromCurrent(amps), alpha,
                amps, rateFit.rSquared(), worstSlipRatio, rateFit.count());

        if (state == IntakeState.STOWED) {
            stowedResult = result;
        } else {
            deployedResult = result;
        }

        System.out.println("[inertia] " + result.describe());
        log(result);
    }

    /**
     * Both configurations, then the report.
     *
     * <p>Needs the robot <b>on the floor with about 2 m of clear space all round</b> — it spins in
     * place, twice, and coasts to a stop between runs.
     *
     * @return the command.
     */
    public Command full() {
        Command stowed = stowIntake == null
                ? spinRun(IntakeState.STOWED)
                : Commands.sequence(
                        Commands.runOnce(stowIntake),
                        Commands.waitSeconds(1.5),
                        spinRun(IntakeState.STOWED));

        Command deployed = deployIntake == null
                ? Commands.runOnce(() -> System.out.println(
                        "[inertia] DEPLOYED skipped — no intake supplied"))
                : Commands.sequence(
                        Commands.runOnce(deployIntake),
                        Commands.waitSeconds(1.5),
                        spinRun(IntakeState.DEPLOYED));

        return Commands.sequence(
                        Commands.runOnce(() -> {
                            stowedResult = null;
                            deployedResult = null;
                            System.out.println("[inertia] === Rotational inertia ===");
                            System.out.println("[inertia] Robot on the floor, about 2 m clear all "
                                    + "round. It will spin in place twice.");
                        }),
                        stowed,
                        deployed,
                        Commands.runOnce(this::printReport))
                .withName("Inertia/Full");
    }

    /** Prints both results and what to do with them. */
    public void printReport() {
        System.out.println();
        System.out.println("=== ROTATIONAL INERTIA REPORT ===");
        System.out.println("I = torque / angular acceleration, both measured on the robot.");
        System.out.println();

        if (stowedResult != null) {
            System.out.println("  " + stowedResult.describe());
        }
        if (deployedResult != null) {
            System.out.println("  " + deployedResult.describe());
        }

        if (stowedResult != null && deployedResult != null
                && stowedResult.isValid() && deployedResult.isValid()) {
            double delta = deployedResult.momentOfInertia() - stowedResult.momentOfInertia();
            double percent = 100 * delta / stowedResult.momentOfInertia();

            System.out.println();
            System.out.printf("  Deploying the intake changes I by %+.3f kg.m^2 (%+.1f%%)%n",
                    delta, percent);

            if (Math.abs(percent) > 10) {
                System.out.println("  ^ more than 10%, so ONE robotMOI cannot describe both states.");
                System.out.println("    PathPlanner will either plan rotations the robot cannot");
                System.out.println("    achieve, or under-drive it. Pick the value for the state your");
                System.out.println("    autos actually run in — usually intake DEPLOYED, since that");
                System.out.println("    is when pieces are being collected. No gain tuning fixes the");
                System.out.println("    other state.");
            } else {
                System.out.println("  ^ under 10%, so a single value describes both well enough.");
            }

            System.out.printf("%n  settings.json currently says robotMOI = 3.733%n");
            System.out.printf("  Suggested: %.3f (deployed) or %.3f (stowed)%n",
                    deployedResult.momentOfInertia(), stowedResult.momentOfInertia());
        }

        System.out.println();
        System.out.println("This is the number CAD normally supplies. Measuring it on the robot");
        System.out.println("includes the wire, tape and fasteners a model never has — but it does");
        System.out.println("depend on the Vortex torque constant and the drive reduction, so it");
        System.out.println("inherits any error in those.");
        System.out.println("=== END ===");
        System.out.println();
    }

    private void log(Result result) {
        String root = "Inertia/" + result.state();
        Logger.recordOutput(root + "/MomentOfInertia", result.momentOfInertia());
        Logger.recordOutput(root + "/TorqueNm", result.torqueNm());
        Logger.recordOutput(root + "/AngularAccelRadPerSec2", result.angularAccelRadPerSec2());
        Logger.recordOutput(root + "/PerMotorAmps", result.perMotorAmps());
        Logger.recordOutput(root + "/RSquared", result.rSquared());
        Logger.recordOutput(root + "/WorstSlipRatio", result.worstSlipRatio());
        Logger.recordOutput(root + "/Valid", result.isValid());
    }

    /** @return the stowed measurement, or null if not run. */
    public Result getStowedResult() {
        return stowedResult;
    }

    /** @return the deployed measurement, or null if not run. */
    public Result getDeployedResult() {
        return deployedResult;
    }

    /** @return true when an intake was supplied, so both states can be measured. */
    public boolean canMeasureBothStates() {
        return hasIntake.getAsBoolean();
    }
}
