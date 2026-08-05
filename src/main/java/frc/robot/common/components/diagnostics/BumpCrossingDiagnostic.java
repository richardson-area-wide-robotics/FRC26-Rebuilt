package frc.robot.common.components.diagnostics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CommonConstants.BatteryConstants;
import frc.robot.CommonConstants.SwerveConstants;
import frc.robot.common.components.diagnostics.VisionCalibration.RunningStats;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Works out why the chassis bogs down going over a ramp.
 *
 * <p>Observed at competition: the robot slows crossing the small field ramps and sometimes fails to
 * get over, despite having far more motor power than the job needs. "Plenty of power" is the clue —
 * the motors are not the constraint, so something between the motors and the carpet is.
 *
 * <p>There are three candidate causes and <b>they need opposite fixes</b>, which is why guessing is
 * expensive:
 *
 * <ul>
 *   <li><b>Current-limited.</b> Climbing needs high torque at low wheel speed, which is where a
 *       brushless motor wants to draw its most current. The drive smart current limit is
 *       {@value frc.robot.CommonConstants.SwerveConstants#DRIVE_MOTOR_CURRENT_LIMIT} A, and the
 *       controller enforces it by cutting duty cycle — so full stick delivers only the torque that
 *       limit allows, however much the motor could produce. <b>Fix: raise the limit.</b>
 *   <li><b>Traction-limited.</b> The wheels are spinning and the robot is not going anywhere. More
 *       current makes this <em>worse</em>, not better. <b>Fix: lower the limit</b>, or change wheels
 *       or weight distribution.
 *   <li><b>Voltage-limited.</b> Four drive motors pulling hard at once sag the pack; less voltage is
 *       less speed and less torque, which needs more current, which sags it further. <b>Fix: battery
 *       management, or a lower limit to stop the compounding.</b>
 * </ul>
 *
 * <p>The three are distinguishable from data the robot already has. <b>Wheel speed against chassis
 * speed</b> separates traction from everything else: if the wheels are turning faster than the robot
 * is moving, they are slipping. <b>Current against the configured limit</b> separates current-limited
 * from not-limited-at-all. <b>Bus voltage</b> catches sag. Nothing here needs a sensor that is not
 * already fitted.
 *
 * <p><b>How to run it.</b> Schedule it, then drive over the ramp normally — it does not take control
 * of the drivetrain, it only watches. Deliberately so: the interesting case is the one the driver
 * experiences, and a canned approach speed would not reproduce it.
 */
public class BumpCrossingDiagnostic {

    /** Seconds of watching. Long enough for an approach, a crossing, and the recovery after it. */
    private static final double WATCH_SECONDS = 6.0;

    /**
     * Fraction of the configured limit that counts as pinned.
     *
     * <p>0.90 rather than 1.0: the smart current limit regulates around its setpoint rather than
     * clipping exactly at it, and the reading is sampled at 50 Hz.
     */
    private static final double PINNED_FRACTION = 0.90;

    /**
     * Wheel-to-chassis speed excess treated as slip, in m/s.
     *
     * <p>Some excess is normal and is not slip — a swerve module's reported speed includes the
     * rotational component, and the pose estimate lags. 0.30 m/s of sustained excess while the robot
     * is barely moving is not explainable that way.
     */
    private static final double SLIP_EXCESS_MPS = 0.30;

    /**
     * Bus voltage below which sag is implicated, in volts.
     *
     * <p>From {@link BatteryConstants#CONCERNING_SAG_VOLTS}, so the number lives with the rest of the
     * voltage facts rather than here.
     *
     * <p>This was 9.5, which was wrong for this robot: it runs 10 to 14 V normally, so a minimum of
     * 9.5 during a hard push is ordinary and the diagnostic would have reported a healthy battery as
     * the cause. Sending someone to the charger while the real limit is elsewhere is worse than saying
     * nothing.
     */
    private static final double SAG_VOLTS = BatteryConstants.CONCERNING_SAG_VOLTS;

    /** What the data says was limiting the robot. */
    public enum Verdict {
        /** Current pinned at the limit, wheels gripping. Raising the limit should help. */
        CURRENT_LIMITED,
        /** Wheels turning faster than the robot moved. More current will not help. */
        TRACTION_LIMITED,
        /** Pack sagged under load. Current headroom is academic until that is fixed. */
        VOLTAGE_LIMITED,
        /** Nothing was at a limit. Look at gearing, the approach, or the ramp geometry. */
        NOT_LIMITED,
        /** The robot barely moved and drew little — the run did not capture a crossing. */
        NO_CROSSING_SEEN,
        /**
         * Nothing else was at a limit, but slip could not be measured, so traction is not ruled out.
         *
         * <p>Distinguishing slip needs a chassis speed that does not come from the wheels, and the
         * only one this robot has is the AprilTag-corrected pose. With no tags in view the pose
         * estimate is pure odometry, which is derived from the wheels — so wheel speed and chassis
         * speed become the same measurement and their difference is zero by construction.
         */
        TRACTION_NOT_MEASURABLE
    }

    /** Everything measured during one crossing. */
    public record Result(
            Verdict verdict,
            double peakAmpsPerMotor,
            double peakTotalAmps,
            double configuredLimitAmps,
            double minBusVolts,
            double peakWheelSpeedMps,
            double peakChassisSpeedMps,
            double maxSlipExcessMps,
            int pinnedLoops,
            int samples,
            boolean slipWasMeasurable) {

        /** @return a readable explanation, including what to do about it. */
        public String describe() {
            String header = String.format(
                    "Bump crossing: peak %.1f A/motor (%.0f A total) against a %.0f A limit, "
                            + "pinned for %d loops; bus fell to %.1f V; wheels peaked at %.2f m/s "
                            + "while the chassis peaked at %.2f m/s (max excess %.2f m/s)",
                    peakAmpsPerMotor, peakTotalAmps, configuredLimitAmps, pinnedLoops,
                    minBusVolts, peakWheelSpeedMps, peakChassisSpeedMps, maxSlipExcessMps);

            String conclusion = switch (verdict) {
                case CURRENT_LIMITED -> String.format(
                        "CURRENT-LIMITED. The wheels gripped and the motors sat at the %.0f A "
                                + "limit, so torque was capped by configuration rather than by "
                                + "physics. Run the traction calibration: if traction does not "
                                + "break until well above %.0f A, that headroom is being left "
                                + "unused and raising the limit is the fix.",
                        configuredLimitAmps, configuredLimitAmps);
                case TRACTION_LIMITED -> String.format(
                        "TRACTION-LIMITED. The wheels turned %.2f m/s faster than the robot "
                                + "moved, so they were slipping. More current makes this worse. "
                                + "Lower the limit to below the traction limit, and look at "
                                + "wheels and weight over the driving corners.",
                        maxSlipExcessMps);
                case VOLTAGE_LIMITED -> String.format(
                        "VOLTAGE-LIMITED. The pack fell to %.1f V under load. Less voltage is "
                                + "less torque, which draws more current, which sags it further — "
                                + "so this compounds. Check the battery and its connections "
                                + "before changing any current limit, because a limit tuned on a "
                                + "tired pack will be wrong on a fresh one.",
                        minBusVolts);
                case NOT_LIMITED -> "NOT LIMITED by current, traction or voltage. The drivetrain "
                        + "had headroom in all three, so look elsewhere: approach speed, the angle "
                        + "of attack, ground clearance, or something fouling on the ramp lip.";
                case TRACTION_NOT_MEASURABLE -> "INCONCLUSIVE. Current and voltage both had "
                        + "headroom, but no AprilTag was in view during the crossing, so chassis "
                        + "speed came from the wheels and slip cannot be detected — the two "
                        + "measurements are the same one. Re-run where a tag is visible before "
                        + "concluding traction is fine.";
                case NO_CROSSING_SEEN -> "NO CROSSING SEEN. The robot barely moved and drew little "
                        + "current, so nothing was captured. Re-run and drive over the ramp during "
                        + "the watch window.";
            };

            return header + "\n  -> " + conclusion;
        }
    }

    private final SwerveDriveSubsystem drive;

    private final RunningStats wheelSpeed = new RunningStats();
    private final RunningStats chassisSpeed = new RunningStats();
    private final RunningStats busVolts = new RunningStats();
    private final RunningStats perMotorAmps = new RunningStats();

    private double maxSlipExcess;
    private int pinnedLoops;

    private Pose2d previousPose = Pose2d.kZero;
    private double previousTimestamp;
    private boolean havePreviousSample;
    private int visionFreshLoops;

    /** Supplies whether vision has accepted a measurement recently. */
    private final BooleanSupplier visionFresh;

    /**
     * @param drive       The drivetrain.
     * @param visionFresh Supplies whether an AprilTag measurement was accepted recently. Without
     *                    tags, slip is undetectable — see {@link Verdict#TRACTION_NOT_MEASURABLE}.
     */
    public BumpCrossingDiagnostic(SwerveDriveSubsystem drive, BooleanSupplier visionFresh) {
        this.drive = drive;
        this.visionFresh = visionFresh;
    }

    /**
     * Builds the watch.
     *
     * <p>Requires nothing, deliberately — it must not interrupt the driver's control of the
     * drivetrain, because the whole point is to measure a crossing as actually driven.
     *
     * @return the diagnostic command.
     */
    public Command watch() {
        return Commands.sequence(
                        Commands.runOnce(() -> {
                            reset();
                            System.out.println("[bump] Watching for " + WATCH_SECONDS
                                    + "s — drive over the ramp now.");
                        }),
                        Commands.run(this::sample).withTimeout(WATCH_SECONDS),
                        Commands.runOnce(() -> System.out.println(analyse().describe())))
                .withName("BumpCrossingDiagnostic");
    }

    /** Clears the accumulated run. */
    public void reset() {
        wheelSpeed.reset();
        chassisSpeed.reset();
        busVolts.reset();
        perMotorAmps.reset();
        maxSlipExcess = 0;
        pinnedLoops = 0;
        havePreviousSample = false;
        visionFreshLoops = 0;
    }

    /** Folds in one loop. */
    private void sample() {
        double wheels = drive.getAverageAbsoluteDriveVelocity();
        double amps = drive.getTotalDriveCurrent() / 4.0;
        double volts = RobotController.getBatteryVoltage();

        // Chassis speed by differentiating the fused pose — NOT from drive.getChassisSpeeds(),
        // which runs kinematics on the module states. That would make wheel speed and chassis speed
        // the same measurement, so slip would read as zero by construction. Which is precisely the
        // mistake this diagnostic exists to avoid making.
        //
        // The fused pose is independent of the wheels only while AprilTags are contributing; with no
        // tags in view it degrades to pure odometry and the circularity comes back. That is tracked
        // rather than assumed away, and reported as TRACTION_NOT_MEASURABLE.
        Pose2d pose = drive.getPose();
        double now = Timer.getFPGATimestamp();
        double chassis = 0;

        if (havePreviousSample) {
            double dt = now - previousTimestamp;
            if (dt > 1e-3) {
                chassis = pose.getTranslation().getDistance(previousPose.getTranslation()) / dt;
            }
        }
        previousPose = pose;
        previousTimestamp = now;
        havePreviousSample = true;

        if (visionFresh.getAsBoolean()) {
            visionFreshLoops++;
        }

        wheelSpeed.add(wheels);
        chassisSpeed.add(chassis);
        busVolts.add(volts);
        perMotorAmps.add(amps);

        maxSlipExcess = Math.max(maxSlipExcess, wheels - chassis);

        if (amps >= SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT * PINNED_FRACTION) {
            pinnedLoops++;
        }

        String root = "BumpDiagnostic";
        Logger.recordOutput(root + "/WheelSpeedMps", wheels);
        Logger.recordOutput(root + "/ChassisSpeedMps", chassis);
        Logger.recordOutput(root + "/SlipExcessMps", wheels - chassis);
        Logger.recordOutput(root + "/AmpsPerMotor", amps);
        Logger.recordOutput(root + "/BusVolts", volts);
        Logger.recordOutput(root + "/PinnedLoops", pinnedLoops);
        Logger.recordOutput(root + "/VisionFreshLoops", visionFreshLoops);
    }

    /** @return the classified result for this run. */
    public Result analyse() {
        // Slip is only measurable if tags were contributing for a decent share of the run. Half is
        // the bar: below that the pose was mostly odometry, so most of the samples the comparison
        // rests on were circular.
        boolean slipMeasurable = perMotorAmps.getCount() > 0
                && visionFreshLoops >= perMotorAmps.getCount() / 2;

        return analyse(perMotorAmps.getMax(), busVolts.getMin(), wheelSpeed.getMax(),
                chassisSpeed.getMax(), maxSlipExcess, pinnedLoops, perMotorAmps.getCount(),
                SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT, slipMeasurable);
    }

    /**
     * Classifies a crossing. Pure, so it is testable without a robot.
     *
     * <p>Order matters. Voltage is checked first because a sagging pack makes the other two
     * measurements untrustworthy — current looks lower than the mechanism wanted and speed looks
     * lower than the gearing allows, so a sag diagnosed as anything else sends someone tuning a
     * limit against a moving target. Traction is checked before current because if the wheels are
     * slipping, the current reading is what it takes to spin them rather than what it takes to climb.
     *
     * @param peakAmpsPerMotor  Highest per-motor current seen.
     * @param minBusVolts       Lowest bus voltage seen.
     * @param peakWheelSpeed    Highest mean absolute wheel speed, m/s.
     * @param peakChassisSpeed  Highest chassis speed, m/s.
     * @param maxSlipExcess     Largest wheel-minus-chassis speed, m/s.
     * @param pinnedLoops       Loops with current at or above the limit.
     * @param samples           Loops recorded.
     * @param limitAmps         Configured per-motor limit.
     * @param slipMeasurable    Whether chassis speed was independent of the wheels during the run.
     * @return the verdict and the numbers behind it.
     */
    static Result analyse(double peakAmpsPerMotor, double minBusVolts, double peakWheelSpeed,
            double peakChassisSpeed, double maxSlipExcess, int pinnedLoops, int samples,
            double limitAmps, boolean slipMeasurable) {

        Verdict verdict;

        if (samples < 25 || (peakWheelSpeed < 0.20 && peakAmpsPerMotor < limitAmps * 0.30)) {
            verdict = Verdict.NO_CROSSING_SEEN;
        } else if (minBusVolts < SAG_VOLTS) {
            verdict = Verdict.VOLTAGE_LIMITED;
        } else if (slipMeasurable && maxSlipExcess > SLIP_EXCESS_MPS) {
            verdict = Verdict.TRACTION_LIMITED;
        } else if (pinnedLoops >= 5) {
            // Current pinned is conclusive whether or not slip was measurable: torque was capped by
            // configuration. Slip would change what to do about it, not whether it happened.
            verdict = Verdict.CURRENT_LIMITED;
        } else if (!slipMeasurable) {
            verdict = Verdict.TRACTION_NOT_MEASURABLE;
        } else {
            verdict = Verdict.NOT_LIMITED;
        }

        return new Result(verdict, peakAmpsPerMotor, peakAmpsPerMotor * 4, limitAmps,
                minBusVolts, peakWheelSpeed, peakChassisSpeed, maxSlipExcess, pinnedLoops,
                samples, slipMeasurable);
    }
}
