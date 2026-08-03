package frc.robot.common.components.diagnostics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CommonConstants.DriveConstants;
import frc.robot.CommonConstants.ModuleConstants;
import frc.robot.common.components.diagnostics.DriveCharacterization.Feedforward;
import frc.robot.common.components.diagnostics.DriveCharacterization.StraightRunResult;
import frc.robot.common.subsystems.drive.DriveStraightClosedLoop;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.common.subsystems.vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Measures the drivetrain's constants against AprilTag ground truth, and reports what they
 * should be.
 *
 * <p>Run from Test mode with the robot on the floor, at least 4 m of clear space ahead, and
 * AprilTags in view. Every routine is a {@link Command} with a timeout, and every speed is
 * deliberately low.
 *
 * <p><b>Measured versus proposed.</b> The wheel scale, gyro scale, effective drive radius,
 * steering misalignment and feedforward are <em>measured</em> — each is a direct comparison
 * between two sensors, one of which is absolute. The closed-loop gains are only
 * <em>proposed</em>: they come from a single step response and ignore dynamics, so they are a
 * starting point to iterate from, not an answer.
 *
 * <p>Nothing is written to source. Results are logged under {@code Calibration/Auto/} and
 * printed as a paste-ready block, and can optionally be applied live for immediate retesting.
 */
public class DriveAutoCalibrator {

    /** Duty cycles swept during feedforward characterisation. */
    private static final double[] FEEDFORWARD_STEPS = {0.10, 0.15, 0.20, 0.30, 0.40, 0.50};

    /** How long to hold each step so velocity reaches steady state, in seconds. */
    private static final double STEP_SETTLE_SECONDS = 1.2;

    /** Straight-run distance used for scale and alignment measurement, in metres. */
    private static final double SCALE_RUN_METERS = 3.0;

    /** Open-loop duty cycle used for the straight scale run. */
    private static final double SCALE_RUN_OUTPUT = 0.20;

    /** Rotation rate for the gyro-scale sweep, as a fraction of maximum. */
    private static final double SPIN_FRACTION = 0.25;

    /** How long to spin, in seconds. Long enough for several turns. */
    private static final double SPIN_SECONDS = 4.0;

    /** The acceptance requirement: 10 feet. */
    public static final double ACCEPTANCE_DISTANCE_METERS = 3.048;

    /** The acceptance tolerance: 1 inch. */
    public static final double ACCEPTANCE_TOLERANCE_METERS = 0.0254;

    private final SwerveDriveSubsystem drive;
    private final VisionSubsystem vision;

    /** Module names in the order every per-module array uses. */
    private static final String[] MODULE_NAMES = {"FL", "FR", "RL", "RR"};

    /**
     * One fit per module rather than one for the whole drivetrain.
     *
     * <p>Averaging the four modules before fitting yields a single tidy kV and conceals the
     * thing worth knowing: published motor specifications are typical values, individual motors
     * vary, and a corner that is materially weaker than its neighbours pulls the robot off a
     * straight line — the same cross-track error the 1 inch budget is fighting.
     */
    private final DriveCharacterization.FeedforwardFit[] moduleFits = {
            new DriveCharacterization.FeedforwardFit(),
            new DriveCharacterization.FeedforwardFit(),
            new DriveCharacterization.FeedforwardFit(),
            new DriveCharacterization.FeedforwardFit()
    };

    // Results.
    private double measuredWheelScale = 1.0;
    private double measuredGyroScale = 1.0;
    private double measuredDriveRadius;
    private double measuredSteerOffsetDegrees;
    private Feedforward measuredFeedforward = new Feedforward(0, 0, 0, 0);
    private final Feedforward[] measuredModuleFeedforwards = new Feedforward[4];
    private DriveCharacterization.ModuleVariance moduleVariance =
            new DriveCharacterization.ModuleVariance(0, 0, 0, 0, -1, 0);
    private StraightRunResult openLoopAcceptance;
    private StraightRunResult closedLoopAcceptance;

    /**
     * @param drive  Drivetrain to calibrate.
     * @param vision Vision subsystem, used for its calibration analyser and tag health.
     */
    public DriveAutoCalibrator(SwerveDriveSubsystem drive, VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
    }

    // -----------------------------------------------------------------------------------
    // Routine 1 — wheel scale and steering alignment, from one straight run
    // -----------------------------------------------------------------------------------

    /**
     * Drives straight open loop and compares encoder distance with AprilTag distance.
     *
     * <p>Open loop deliberately: closed-loop position control would correct the very error
     * being measured. The same run also yields the common-mode steering misalignment, since
     * the direction actually travelled can be compared with the direction commanded.
     *
     * @return a command that performs the run and records the results.
     */
    public Command measureWheelScaleAndAlignment() {
        double[] startEncoder = new double[1];
        Pose2d[] startPose = new Pose2d[1];
        double[] startHeading = new double[1];

        return Commands.sequence(
                Commands.runOnce(() -> {
                    startEncoder[0] = drive.getAverageDriveDistance();
                    startPose[0] = drive.getPose();
                    startHeading[0] = drive.getHeading();
                    System.out.println("[calib] straight run: " + SCALE_RUN_METERS + " m open loop");
                }),
                // Drive open loop until the encoders say we have gone far enough. Encoder
                // distance is the wrong measure of truth, but it is a perfectly good way to
                // decide when to stop.
                Commands.run(() -> drive.driveOpenLoop(SCALE_RUN_OUTPUT), drive)
                        .until(() -> Math.abs(drive.getAverageDriveDistance() - startEncoder[0])
                                >= SCALE_RUN_METERS)
                        .withTimeout(10.0),
                Commands.runOnce(() -> drive.drive(0, 0, 0, false), drive),
                // Let the pose estimate settle on fresh tag data before reading it.
                Commands.waitSeconds(1.0),
                Commands.runOnce(() -> {
                    double encoderDistance =
                            Math.abs(drive.getAverageDriveDistance() - startEncoder[0]);
                    Translation2d displacement =
                            drive.getPose().getTranslation().minus(startPose[0].getTranslation());
                    double tagDistance = displacement.getNorm();

                    measuredWheelScale =
                            DriveCharacterization.wheelScale(tagDistance, encoderDistance);

                    // Direction actually travelled, versus the heading we held.
                    double travelDirection = Math.toDegrees(
                            Math.atan2(displacement.getY(), displacement.getX()));
                    measuredSteerOffsetDegrees =
                            DriveCharacterization.commonModeSteerOffsetDegrees(
                                    startHeading[0], travelDirection);

                    double crossTrackAt10ft = DriveCharacterization
                            .crossTrackErrorFromSteerOffset(
                                    measuredSteerOffsetDegrees, ACCEPTANCE_DISTANCE_METERS);

                    System.out.printf(
                            "[calib] encoder %.3f m, tags %.3f m -> wheel scale %.4f%n",
                            encoderDistance, tagDistance, measuredWheelScale);
                    System.out.printf(
                            "[calib] steering offset %.3f deg -> %.1f mm cross-track over 10 ft%n",
                            measuredSteerOffsetDegrees, crossTrackAt10ft * 1000.0);

                    Logger.recordOutput("Calibration/Auto/WheelScale", measuredWheelScale);
                    Logger.recordOutput("Calibration/Auto/EncoderDistance", encoderDistance);
                    Logger.recordOutput("Calibration/Auto/TagDistance", tagDistance);
                    Logger.recordOutput("Calibration/Auto/SteerOffsetDeg",
                            measuredSteerOffsetDegrees);
                    Logger.recordOutput("Calibration/Auto/SteerCrossTrackAt10ftMm",
                            crossTrackAt10ft * 1000.0);
                }))
                .withName("MeasureWheelScaleAndAlignment");
    }

    // -----------------------------------------------------------------------------------
    // Routine 2 — gyro scale and effective drive radius, from one spin
    // -----------------------------------------------------------------------------------

    /**
     * Spins in place and compares gyro heading change with AprilTag heading change, while also
     * deriving the effective drive radius from module speed against rotation rate.
     *
     * <p>Several full turns rather than one, so the scale error is large compared with tag
     * measurement noise. Heading is accumulated continuously because a single before-and-after
     * comparison cannot tell 10 degrees from 370.
     *
     * @return a command that performs the spin and records the results.
     */
    public Command measureGyroScaleAndRadius() {
        double[] gyroAccumulated = new double[1];
        double[] tagAccumulated = new double[1];
        double[] lastGyro = new double[1];
        double[] lastTag = new double[1];
        double[] speedSum = new double[1];
        double[] rateSum = new double[1];
        int[] samples = new int[1];

        return Commands.sequence(
                Commands.runOnce(() -> {
                    gyroAccumulated[0] = 0;
                    tagAccumulated[0] = 0;
                    lastGyro[0] = drive.getHeading();
                    lastTag[0] = drive.getPose().getRotation().getDegrees();
                    speedSum[0] = 0;
                    rateSum[0] = 0;
                    samples[0] = 0;
                    System.out.println("[calib] spinning in place for gyro scale and radius");
                }),
                Commands.run(() -> {
                    drive.drive(0, 0, SPIN_FRACTION, false);

                    // Unwrap both heading sources so multiple turns accumulate.
                    double gyro = drive.getHeading();
                    double tag = drive.getPose().getRotation().getDegrees();
                    gyroAccumulated[0] += shortestDelta(lastGyro[0], gyro);
                    tagAccumulated[0] += shortestDelta(lastTag[0], tag);
                    lastGyro[0] = gyro;
                    lastTag[0] = tag;

                    speedSum[0] += drive.getAverageAbsoluteDriveVelocity();
                    rateSum[0] += Math.toRadians(drive.getTurnRate());
                    samples[0]++;
                }, drive).withTimeout(SPIN_SECONDS),
                Commands.runOnce(() -> drive.drive(0, 0, 0, false), drive),
                Commands.waitSeconds(1.0),
                Commands.runOnce(() -> {
                    measuredGyroScale = DriveCharacterization.gyroScale(
                            tagAccumulated[0], gyroAccumulated[0]);

                    if (samples[0] > 0) {
                        double meanSpeed = speedSum[0] / samples[0];
                        double meanRate = rateSum[0] / samples[0];
                        measuredDriveRadius =
                                DriveCharacterization.effectiveDriveRadius(meanSpeed, meanRate);
                    }

                    double geometric = DriveCharacterization.geometricDriveRadius(
                            DriveConstants.kTrackWidth, DriveConstants.kWheelBase);

                    System.out.printf(
                            "[calib] gyro %.1f deg, tags %.1f deg -> gyro scale %.4f%n",
                            gyroAccumulated[0], tagAccumulated[0], measuredGyroScale);
                    System.out.printf(
                            "[calib] drive radius measured %.4f m, geometric %.4f m (%.1f%%)%n",
                            measuredDriveRadius, geometric,
                            geometric > 0 ? (measuredDriveRadius / geometric - 1) * 100 : 0);

                    Logger.recordOutput("Calibration/Auto/GyroScale", measuredGyroScale);
                    Logger.recordOutput("Calibration/Auto/GyroAccumulatedDeg", gyroAccumulated[0]);
                    Logger.recordOutput("Calibration/Auto/TagAccumulatedDeg", tagAccumulated[0]);
                    Logger.recordOutput("Calibration/Auto/DriveRadiusMeasured",
                            measuredDriveRadius);
                    Logger.recordOutput("Calibration/Auto/DriveRadiusGeometric", geometric);
                }))
                .withName("MeasureGyroScaleAndRadius");
    }

    /** Shortest signed angular difference from a to b, in degrees. */
    private static double shortestDelta(double fromDegrees, double toDegrees) {
        double delta = (toDegrees - fromDegrees) % 360.0;
        if (delta > 180) {
            delta -= 360;
        } else if (delta < -180) {
            delta += 360;
        }
        return delta;
    }

    // -----------------------------------------------------------------------------------
    // Routine 3 — feedforward
    // -----------------------------------------------------------------------------------

    /**
     * Sweeps open-loop duty cycles and fits {@code V = kS + kV * v}.
     *
     * <p>Needs about 4 m of space: the sweep reaches roughly half speed. Each step is held long
     * enough for velocity to settle before it is recorded, because the fit assumes steady
     * state.
     *
     * @return a command that performs the sweep and records the fit.
     */
    public Command measureFeedforward() {
        Command sequence = Commands.runOnce(() -> {
            for (DriveCharacterization.FeedforwardFit fit : moduleFits) {
                fit.reset();
            }
            System.out.println("[calib] feedforward sweep: " + FEEDFORWARD_STEPS.length
                    + " steps, fitting each module separately");
        });

        for (double step : FEEDFORWARD_STEPS) {
            sequence = sequence
                    .andThen(Commands.run(() -> drive.driveOpenLoop(step), drive)
                            .withTimeout(STEP_SETTLE_SECONDS))
                    .andThen(Commands.runOnce(() -> {
                        double[] volts = drive.getModuleDriveVoltages();
                        double[] velocities = drive.getModuleDriveVelocities();
                        for (int i = 0; i < moduleFits.length; i++) {
                            moduleFits[i].add(volts[i], velocities[i]);
                        }
                        System.out.printf(
                                "[calib]   %.2f duty -> FL %.3f  FR %.3f  RL %.3f  RR %.3f m/s%n",
                                step, velocities[0], velocities[1], velocities[2], velocities[3]);
                    }))
                    // Coast back to rest between steps so each one starts from the same place.
                    .andThen(Commands.runOnce(() -> drive.driveOpenLoop(0), drive))
                    .andThen(Commands.waitSeconds(0.6));
        }

        return sequence
                .andThen(Commands.runOnce(this::fitFeedforward))
                .withName("MeasureFeedforward");
    }

    /** Fits each module, then summarises how much the four disagree. */
    private void fitFeedforward() {
        for (int i = 0; i < moduleFits.length; i++) {
            measuredModuleFeedforwards[i] = moduleFits[i].fit();

            Feedforward fit = measuredModuleFeedforwards[i];
            System.out.printf("[calib]   %s: kS %.4f V, kV %.4f V/(m/s), R2 %.4f%s%n",
                    MODULE_NAMES[i], fit.kS(), fit.kV(), fit.rSquared(),
                    fit.isTrustworthy() ? "" : "  POOR FIT");

            String root = "Calibration/Auto/Module/" + MODULE_NAMES[i];
            Logger.recordOutput(root + "/kS", fit.kS());
            Logger.recordOutput(root + "/kV", fit.kV());
            Logger.recordOutput(root + "/R2", fit.rSquared());
            Logger.recordOutput(root + "/Trustworthy", fit.isTrustworthy());
        }

        moduleVariance = DriveCharacterization.summariseModuleVariance(measuredModuleFeedforwards);

        // The robot-wide figure is the mean of the usable module fits, so it stays meaningful
        // while the per-module numbers remain available.
        double meanKs = 0;
        int usable = 0;
        for (Feedforward fit : measuredModuleFeedforwards) {
            if (fit != null && fit.isTrustworthy()) {
                meanKs += fit.kS();
                usable++;
            }
        }
        measuredFeedforward = usable == 0
                ? new Feedforward(0, 0, 0, 0)
                : new Feedforward(meanKs / usable, moduleVariance.meanKv(), 1.0, usable);

        System.out.printf("[calib] module kV spread %.1f%% (%.4f to %.4f), worst %s%n",
                moduleVariance.spreadPercent(), moduleVariance.minKv(), moduleVariance.maxKv(),
                moduleVariance.worstModule() >= 0
                        ? MODULE_NAMES[moduleVariance.worstModule()] : "n/a");

        if (!moduleVariance.isUniform() && moduleVariance.usableFits() >= 3) {
            System.out.println("[calib] NOTE modules differ by more than 8% — published motor "
                    + "specs are typical values and individual motors vary, but this much spread "
                    + "means one corner is materially weaker. Check gearing and wheel wear on "
                    + (moduleVariance.worstModule() >= 0
                            ? MODULE_NAMES[moduleVariance.worstModule()] : "the outlier")
                    + " before accepting a single robot-wide kV.");
        }

        Logger.recordOutput("Calibration/Auto/kS", measuredFeedforward.kS());
        Logger.recordOutput("Calibration/Auto/kV", measuredFeedforward.kV());
        Logger.recordOutput("Calibration/Auto/ModuleKvSpreadPercent",
                moduleVariance.spreadPercent());
        Logger.recordOutput("Calibration/Auto/ModuleKvUniform", moduleVariance.isUniform());
        Logger.recordOutput("Calibration/Auto/ModuleKvUsableFits", moduleVariance.usableFits());
    }

    // -----------------------------------------------------------------------------------
    // Routine 4 — the acceptance test, both ways
    // -----------------------------------------------------------------------------------

    /**
     * Drives 10 feet open loop and reports how far off it finished.
     *
     * <p>This is the honest measure of dead-reckoning accuracy, and the number the requirement
     * is really about when no correction is available — during a path where tags are not
     * visible, for instance.
     *
     * @return a command that performs the run and records the result.
     */
    public Command acceptanceRunOpenLoop() {
        double[] startEncoder = new double[1];
        Pose2d[] startPose = new Pose2d[1];
        double[] startHeading = new double[1];

        return Commands.sequence(
                Commands.runOnce(() -> {
                    startEncoder[0] = drive.getAverageDriveDistance();
                    startPose[0] = drive.getPose();
                    startHeading[0] = drive.getHeading();
                    System.out.println("[calib] ACCEPTANCE open loop: 10 ft");
                }),
                Commands.run(() -> drive.driveOpenLoop(SCALE_RUN_OUTPUT), drive)
                        .until(() -> Math.abs(drive.getAverageDriveDistance() - startEncoder[0])
                                >= ACCEPTANCE_DISTANCE_METERS)
                        .withTimeout(12.0),
                Commands.runOnce(() -> drive.drive(0, 0, 0, false), drive),
                Commands.waitSeconds(1.5),
                Commands.runOnce(() -> {
                    Translation2d displacement =
                            drive.getPose().getTranslation().minus(startPose[0].getTranslation());
                    openLoopAcceptance = DriveCharacterization.analyseStraightRun(
                            ACCEPTANCE_DISTANCE_METERS,
                            startHeading[0],
                            displacement.getX(),
                            displacement.getY(),
                            shortestDelta(startHeading[0], drive.getHeading()));
                    reportAcceptance("OpenLoop", openLoopAcceptance);
                }))
                .withName("AcceptanceOpenLoop");
    }

    /**
     * Drives 10 feet with continuous AprilTag and gyro correction, and reports the error.
     *
     * <p>Expected to be substantially better than the open-loop run, because terminating on the
     * fused pose removes wheel-scale error from the endpoint and actively steering onto the line
     * stops steering misalignment integrating.
     *
     * @return a command that performs the run and records the result.
     */
    public Command acceptanceRunClosedLoop() {
        Pose2d[] startPose = new Pose2d[1];
        double[] startHeading = new double[1];

        return Commands.sequence(
                Commands.runOnce(() -> {
                    startPose[0] = drive.getPose();
                    startHeading[0] = drive.getHeading();
                    System.out.println("[calib] ACCEPTANCE closed loop: 10 ft with correction");
                }),
                new DriveStraightClosedLoop(drive, ACCEPTANCE_DISTANCE_METERS).withTimeout(15.0),
                Commands.waitSeconds(1.5),
                Commands.runOnce(() -> {
                    Translation2d displacement =
                            drive.getPose().getTranslation().minus(startPose[0].getTranslation());
                    closedLoopAcceptance = DriveCharacterization.analyseStraightRun(
                            ACCEPTANCE_DISTANCE_METERS,
                            startHeading[0],
                            displacement.getX(),
                            displacement.getY(),
                            shortestDelta(startHeading[0], drive.getHeading()));
                    reportAcceptance("ClosedLoop", closedLoopAcceptance);
                }))
                .withName("AcceptanceClosedLoop");
    }

    private void reportAcceptance(String label, StraightRunResult result) {
        boolean pass = result.meetsTolerance(ACCEPTANCE_TOLERANCE_METERS);

        System.out.printf(
                "[calib] %s: along %+.1f mm, cross %+.1f mm, total %.1f mm, heading %+.2f deg -> %s%n",
                label,
                result.alongTrackErrorMeters() * 1000.0,
                result.crossTrackErrorMeters() * 1000.0,
                result.totalErrorMeters() * 1000.0,
                result.headingErrorDegrees(),
                pass ? "PASS (within 1 inch)" : "FAIL (over 1 inch)");

        String root = "Calibration/Auto/Acceptance/" + label;
        Logger.recordOutput(root + "/AlongErrorMm", result.alongTrackErrorMeters() * 1000.0);
        Logger.recordOutput(root + "/CrossErrorMm", result.crossTrackErrorMeters() * 1000.0);
        Logger.recordOutput(root + "/TotalErrorMm", result.totalErrorMeters() * 1000.0);
        Logger.recordOutput(root + "/HeadingErrorDeg", result.headingErrorDegrees());
        Logger.recordOutput(root + "/Pass", pass);
    }

    // -----------------------------------------------------------------------------------
    // Full sequence and reporting
    // -----------------------------------------------------------------------------------

    /**
     * The whole calibration, in dependency order.
     *
     * <p>Wheel scale first, because the feedforward fit measures velocity in metres per second
     * and that unit is only correct once the wheel scale is. The acceptance runs come last so
     * they reflect everything measured before them.
     *
     * @return a command that runs every routine and prints the report.
     */
    public Command full() {
        return Commands.sequence(
                Commands.runOnce(this::announceStart),
                Commands.either(
                        Commands.sequence(
                                measureWheelScaleAndAlignment(),
                                measureGyroScaleAndRadius(),
                                measureFeedforward(),
                                acceptanceRunOpenLoop(),
                                acceptanceRunClosedLoop()),
                        Commands.runOnce(() -> System.out.println(
                                "[calib] ABORTED: no AprilTag measurements. Nothing here means "
                                        + "anything without ground truth — check the camera name "
                                        + "and that tags are in view.")),
                        vision::hasRecentMeasurement),
                Commands.runOnce(this::printReport))
                .withName("DriveAutoCalibration");
    }

    private void announceStart() {
        System.out.println("=====================================================");
        System.out.println(" Drivetrain auto-calibration");
        System.out.println(" Needs ~4 m clear ahead and AprilTags in view.");
        System.out.println("=====================================================");
    }

    /** Prints a paste-ready summary of everything measured. */
    public void printReport() {
        double correctedDiameter = ModuleConstants.kWheelDiameterMeters * measuredWheelScale;

        System.out.println("=====================================================");
        System.out.println(" CALIBRATION REPORT");
        System.out.println("-----------------------------------------------------");
        System.out.println(" MEASURED — safe to apply");
        System.out.printf("   kWheelDiameterMeters = %.6f;  // was %.6f, scale %.4f%n",
                correctedDiameter, ModuleConstants.kWheelDiameterMeters, measuredWheelScale);
        System.out.printf("   kDrivingMotorFreeSpeedRps stays %.2f (motor datasheet)%n",
                ModuleConstants.kDrivingMotorFreeSpeedRps);
        System.out.printf("   gyro scale factor    = %.4f%n", measuredGyroScale);
        System.out.printf("   effective drive radius = %.4f m (geometric %.4f m)%n",
                measuredDriveRadius,
                DriveCharacterization.geometricDriveRadius(
                        DriveConstants.kTrackWidth, DriveConstants.kWheelBase));
        System.out.printf("   common-mode steer offset = %.3f deg%n", measuredSteerOffsetDegrees);
        System.out.printf("   kS = %.4f V, kV = %.4f V/(m/s)  [mean of %d module fits]%n",
                measuredFeedforward.kS(), measuredFeedforward.kV(),
                moduleVariance.usableFits());
        System.out.println("   per-module kV — published specs are typical, motors vary:");
        for (int i = 0; i < measuredModuleFeedforwards.length; i++) {
            Feedforward fit = measuredModuleFeedforwards[i];
            if (fit == null) {
                continue;
            }
            System.out.printf("     %s  kV %.4f  kS %.4f  R2 %.4f%s%n",
                    MODULE_NAMES[i], fit.kV(), fit.kS(), fit.rSquared(),
                    fit.isTrustworthy() ? "" : "  POOR FIT");
        }
        System.out.printf("   spread %.1f%% -> %s%n", moduleVariance.spreadPercent(),
                moduleVariance.isUniform()
                        ? "close enough to share one kV"
                        : "one corner is materially weaker; investigate before averaging");
        System.out.println("-----------------------------------------------------");
        System.out.println(" ACCEPTANCE — 10 ft, 1 inch budget");
        if (openLoopAcceptance != null) {
            System.out.printf("   open loop   total %.1f mm  %s%n",
                    openLoopAcceptance.totalErrorMeters() * 1000.0,
                    openLoopAcceptance.meetsTolerance(ACCEPTANCE_TOLERANCE_METERS)
                            ? "PASS" : "FAIL");
        }
        if (closedLoopAcceptance != null) {
            System.out.printf("   closed loop total %.1f mm  %s%n",
                    closedLoopAcceptance.totalErrorMeters() * 1000.0,
                    closedLoopAcceptance.meetsTolerance(ACCEPTANCE_TOLERANCE_METERS)
                            ? "PASS" : "FAIL");
        }
        System.out.println("=====================================================");
    }

    /** @return the measured wheel-diameter correction factor. */
    public double getWheelScale() {
        return measuredWheelScale;
    }

    /** @return the measured gyro scale correction factor. */
    public double getGyroScale() {
        return measuredGyroScale;
    }

    /** @return the measured common-mode steering misalignment, in degrees. */
    public double getSteerOffsetDegrees() {
        return measuredSteerOffsetDegrees;
    }

    /** @return the robot-wide feedforward, being the mean of the usable module fits. */
    public Feedforward getFeedforward() {
        return measuredFeedforward;
    }

    /** @return per-module feedforward fits in FL, FR, RL, RR order. Entries may be null. */
    public Feedforward[] getModuleFeedforwards() {
        return measuredModuleFeedforwards.clone();
    }

    /** @return how much the four modules disagree about their own feedforward. */
    public DriveCharacterization.ModuleVariance getModuleVariance() {
        return moduleVariance;
    }

    /** @return the open-loop acceptance result, or null if it has not run. */
    public StraightRunResult getOpenLoopAcceptance() {
        return openLoopAcceptance;
    }

    /** @return the closed-loop acceptance result, or null if it has not run. */
    public StraightRunResult getClosedLoopAcceptance() {
        return closedLoopAcceptance;
    }
}
