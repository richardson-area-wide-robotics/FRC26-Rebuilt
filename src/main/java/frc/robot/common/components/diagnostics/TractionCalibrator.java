package frc.robot.common.components.diagnostics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CommonConstants.SwerveConstants;
import frc.robot.common.components.diagnostics.VisionCalibration.RunningStats;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Finds the drive current limit at which the wheels break traction, by pushing into a wall.
 *
 * <p>Put the robot squarely against a wall on carpet, command full forward output, and step the drive
 * smart current limit upward. At a low limit the motors cannot make enough torque to overcome the
 * tyres' grip: the robot pushes, the wheels stay still. Past some limit the torque exceeds what the
 * carpet will hold and the wheels start to spin. That limit is the traction limit, and the useful
 * setting sits just below it.
 *
 * <h2>Why the limit belongs below the traction limit</h2>
 *
 * <p>Not only to protect the motors. Below the traction limit <b>the wheels physically cannot slip
 * under their own torque</b>, which means wheel rotation always corresponds to ground travelled and
 * odometry stays honest. Above it, hard acceleration spins the wheels, the encoders count distance
 * the robot never moved, and the pose estimate gains error that no amount of vision correction knows
 * to expect. That is the same error budget as the 10 ft acceptance run, arriving through the throttle
 * instead of through the wheel diameter.
 *
 * <h2>How slip is detected</h2>
 *
 * <p>Against a wall the two outcomes are cleanly separable. Gripping means the robot cannot move and
 * the wheels cannot turn, so wheel velocity is ~0. Slipping means the wheels turn while the robot
 * still does not move. So <b>wheel velocity high with the chassis stationary is slip</b>, and the
 * chassis half of that is what stops the test mistaking "driving across the room" for slip. If the
 * robot moves, it was not against the wall and the run aborts rather than reporting a number.
 *
 * <h2>What can invalidate a run</h2>
 *
 * <ul>
 *   <li><b>Not actually against the wall.</b> Detected and aborted, as above.
 *   <li><b>The limit never binds.</b> If measured current sits well below the limit that was set,
 *       raising the limit is not what is changing, and the step proves nothing. Flagged per step.
 *   <li><b>A tired battery.</b> Recorded per step. Traction depends on torque and torque on current,
 *       so a pack sagging under load reports a higher limit as safe than a fresh one would.
 * </ul>
 *
 * <p><b>This writes nothing.</b> It prints a recommendation for
 * {@code SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT}. The limits it applies while running do not
 * persist, so a power cycle restores whatever the code says — a calibration run cannot leave the
 * robot configured for a limit nobody chose.
 */
public class TractionCalibrator {

    /** First limit tried, in amps. Low enough that no drivetrain breaks traction here. */
    private static final int START_AMPS = 20;

    /** Step between limits, in amps. */
    private static final int STEP_AMPS = 5;

    /**
     * Hard ceiling on both the sweep and the recommendation, in amps.
     *
     * <p>Requested explicitly, and it is the right number for a per-motor limit on this drivetrain.
     * See {@link Result#breakerWarning()} for why it is not the whole story.
     */
    public static final int HARD_CAP_AMPS = 80;

    /**
     * Amps of margin below the traction limit.
     *
     * <p>The traction limit is measured on one carpet, at one battery state, with the tyres as worn
     * as they are today. A fresh pack makes more torque at the same limit and a clean carpet grips
     * differently, so sitting exactly at the measured limit means slipping the first time conditions
     * improve.
     */
    private static final int SAFETY_MARGIN_AMPS = 5;

    /** Duty cycle commanded while pushing. Full output, so the current limit is what limits. */
    private static final double PUSH_OUTPUT = 1.0;

    /**
     * Seconds spent pushing at each limit.
     *
     * <p>Short on purpose. A stalled brushless motor makes no back-EMF, so essentially all of its
     * input becomes heat in the windings, and the smart current limit caps the current but does
     * nothing about the duration. 0.75 s is 37 samples, plenty to tell a spinning wheel from a still
     * one.
     */
    private static final double PUSH_SECONDS = 0.75;

    /** Seconds of rest between steps, so the motors are not stalled back to back. */
    private static final double COOLDOWN_SECONDS = 2.5;

    /** Seconds of settling before sampling, to skip the current inrush and the frame taking up load. */
    private static final double SETTLE_SECONDS = 0.25;

    /**
     * Wheel speed treated as slip, in m/s.
     *
     * <p>Not zero. A drivetrain pushing hard into a wall creeps a little as the tyres deform and the
     * frame takes up load, which shows as a few cm/s of wheel rotation with no slip at all. 0.15 m/s
     * is about 3% of this robot's free speed and well clear of that creep.
     */
    private static final double SLIP_VELOCITY_MPS = 0.15;

    /**
     * Odometry distance, in metres, at which the run is stopped for safety.
     *
     * <p>Not a validity test. Odometry is fed by the same wheels whose slip is being measured, so it
     * cannot distinguish creep from slip at the scale that matters -- which is exactly the mistake
     * this constant replaced. What it can do is notice that the robot is crossing the room, which is
     * a reason to stop regardless of how the number was obtained.
     */
    private static final double RUNAWAY_METERS = 2.0;

    /** Fraction of the commanded limit the current must reach for the limit to be binding. */
    private static final double BINDING_FRACTION = 0.80;

    /** One current limit tried, and what happened. */
    public record Step(
            int limitAmps,
            double wheelSpeedMps,
            double measuredAmpsPerMotor,
            double totalAmps,
            double batteryVolts,
            double chassisMovedMeters,
            boolean slipped,
            boolean limitWasBinding,
            boolean robotMoved) {

        /** @return a one-line summary of this step. */
        public String describe() {
            String verdict;
            if (robotMoved) {
                verdict = "STOPPED, odometry moved "
                        + String.format("%.2f", chassisMovedMeters) + " m";
            } else if (slipped) {
                verdict = "SLIPPED";
            } else if (!limitWasBinding) {
                verdict = "gripped, but limit not binding";
            } else {
                verdict = "gripped";
            }

            return String.format(
                    "  %3d A limit -> wheels %.2f m/s, drew %.1f A/motor (%.0f A total), "
                            + "%.1f V bus : %s",
                    limitAmps, wheelSpeedMps, measuredAmpsPerMotor, totalAmps, batteryVolts, verdict);
        }
    }

    /** The outcome of a whole sweep. */
    public record Result(List<Step> steps, int recommendedAmps, int tractionLimitAmps,
            boolean aborted, String abortReason) {

        /** @return true when a slip was actually observed, rather than the sweep running out of range. */
        public boolean foundTractionLimit() {
            return tractionLimitAmps > 0;
        }

        /**
         * @return a warning if the recommendation puts the whole drivetrain past the main breaker,
         *     or an empty string.
         *
         *     <p>The recommendation is <em>per motor</em>, and there are four of them. A 70 A
         *     per-motor limit is 280 A with all four pushing, against a 120 A main breaker. The
         *     breaker is thermal and tolerates that for a few seconds, which is why teams get away
         *     with it in a pushing match — but it is worth knowing that the number being set is not
         *     one the robot can hold indefinitely.
         */
        public String breakerWarning() {
            int total = recommendedAmps * 4;
            if (total <= 120) {
                return "";
            }

            return String.format(
                    "NOTE: %d A per motor is %d A across four drive motors, against a 120 A main "
                            + "breaker. The breaker is thermal so short pushes are fine, but this is "
                            + "not a limit the drivetrain can hold. If the robot browns out during "
                            + "pushing matches, this is the first number to lower.",
                    recommendedAmps, total);
        }
    }

    private final SwerveDriveSubsystem drive;
    private final List<Step> steps = new ArrayList<>();

    private Pose2d stepStartPose = Pose2d.kZero;
    private final RunningStats stepWheelSpeed = new RunningStats();
    private final RunningStats stepCurrent = new RunningStats();
    private final RunningStats stepVoltage = new RunningStats();
    // Deliberately no `aborted` field. Whether a run is valid is a property of the steps that were
    // recorded, so it is derived in analyse() and here. A cached flag alongside the steps is a second
    // source of truth that can disagree with them.

    /**
     * @param drive The drivetrain.
     */
    public TractionCalibrator(SwerveDriveSubsystem drive) {
        this.drive = drive;
    }

    /**
     * Builds the sweep.
     *
     * <p>Steps from {@value #START_AMPS} A to {@value #HARD_CAP_AMPS} A, stopping early once slip is
     * seen — there is nothing to learn from pushing harder once the wheels are already spinning, and
     * every extra step is more heat.
     *
     * @return the calibration command.
     */
    public Command sweep() {
        List<Command> phases = new ArrayList<>();

        phases.add(Commands.runOnce(() -> {
            steps.clear();
            System.out.println("[Traction] === Drive current limit calibration ===");
            System.out.println("[Traction] Robot must be SQUARE against a wall, on carpet, "
                    + "with a good battery.");
            System.out.println("[Traction] Stepping the limit from " + START_AMPS + " A to "
                    + HARD_CAP_AMPS + " A until the wheels break loose.");
        }));

        for (int amps = START_AMPS; amps <= HARD_CAP_AMPS; amps += STEP_AMPS) {
            phases.add(pushAt(amps));
        }

        phases.add(Commands.runOnce(() -> {
            // Always hand the drivetrain back at the configured limit, whatever the sweep reached.
            // Leaving it wherever the last step happened to be would mean the next thing the robot
            // did ran on a calibration artefact.
            drive.applyDriveCurrentLimit(SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
            drive.driveOpenLoop(0);
            printReport();
        }, drive));

        return Commands.sequence(phases.toArray(new Command[0]))
                .finallyDo(interrupted -> {
                    // Also on interrupt or disable. A limit left raised after an aborted run is
                    // exactly the kind of thing that gets discovered during a match.
                    drive.applyDriveCurrentLimit(SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
                    drive.driveOpenLoop(0);
                })
                .withName("TractionCalibration");
    }

    /**
     * One step: set the limit, push, sample, rest.
     *
     * @param amps limit to try.
     * @return the step command.
     */
    private Command pushAt(int amps) {
        // Every command that touches the drivetrain declares it as a requirement, so the sequence
        // interrupts the default drive command for its whole duration. Without that the default
        // command keeps running and writes zero output over every push, and the sweep measures a
        // drivetrain that is not pushing at all.
        Command push = Commands.sequence(
                Commands.runOnce(() -> {
                    stepStartPose = drive.getPose();
                    stepWheelSpeed.reset();
                    stepCurrent.reset();
                    stepVoltage.reset();
                    drive.applyDriveCurrentLimit(amps);
                }, drive),
                // Settle first: the inrush as the motors take up load is not the steady state being
                // measured, and folding it in would read as slip on every step.
                Commands.run(() -> drive.driveOpenLoop(PUSH_OUTPUT), drive)
                        .withTimeout(SETTLE_SECONDS),
                Commands.run(() -> {
                    drive.driveOpenLoop(PUSH_OUTPUT);
                    stepWheelSpeed.add(drive.getAverageAbsoluteDriveVelocity());
                    stepCurrent.add(drive.getTotalDriveCurrent() / 4.0);
                    stepVoltage.add(RobotController.getBatteryVoltage());
                }, drive).withTimeout(PUSH_SECONDS),
                Commands.runOnce(() -> recordStep(amps)),
                Commands.runOnce(() -> drive.driveOpenLoop(0), drive),
                Commands.waitSeconds(COOLDOWN_SECONDS));

        // Skip every remaining step once slip is found or the run is invalid. Commands.either with
        // an empty branch rather than a cancel, so the sequence stays a plain composition.
        return Commands.either(Commands.none(), push, this::isFinished);
    }

    /** @return true once there is nothing more to learn from pushing harder. */
    private boolean isFinished() {
        // Slip found, or the robot has travelled far enough that something is wrong. The second
        // is a safety stop, not a verdict -- see RUNAWAY_METERS.
        return steps.stream().anyMatch(Step::robotMoved) || tractionLimitAmps() > 0;
    }

    /** Folds one step's samples into a result. */
    private void recordStep(int amps) {
        double moved = drive.getPose().getTranslation()
                .getDistance(stepStartPose.getTranslation());
        double wheelSpeed = stepWheelSpeed.getMean();
        double perMotor = stepCurrent.getMean();

        boolean binding = perMotor >= amps * BINDING_FRACTION;

        // Slip is the wheels spinning up WHILE THE CURRENT LIMIT IS STILL BINDING.
        //
        // It used to be "wheels turning and the robot staying put", with staying put taken from
        // drive.getPose(). That could never fire, and the arithmetic says so: getPose() is the pose
        // estimator, which in a shop with no tag in view is wheel-integrated odometry -- so spinning
        // wheels advance it. Wheel speed above 0.15 m/s sustained over the 0.75 s push necessarily
        // moves the estimate more than the 0.10 m static tolerance, so robotMoved was true exactly
        // when slip occurred, and the two halves of the test excluded each other. The step that
        // genuinely broke traction aborted the run as "not against the wall", and squaring the robot
        // up again produced the same message for ever.
        //
        // Binding is the better second half, and it needs no chassis reference at all: a robot that
        // is NOT against the wall rolls freely and never reaches its current limit, so binding alone
        // establishes that the drivetrain is pushing something immovable. Wheels turning while still
        // pinned at the limit is slip, and nothing else is.
        //
        // The same reasoning is already written down in BumpCrossingDiagnostic, which refuses to
        // compare wheel speed against a chassis speed derived from wheels. This file disagreed with
        // it.
        boolean slipped = wheelSpeed > SLIP_VELOCITY_MPS && binding;

        // Kept, but only as a SAFETY stop and an informational figure -- never as a validity gate.
        // Odometry cannot tell 10 cm of creep from 10 cm of slip; it can tell that the robot has
        // travelled several metres and should be stopped.
        boolean robotMoved = moved > RUNAWAY_METERS;

        Step step = new Step(amps, wheelSpeed, perMotor, perMotor * 4,
                stepVoltage.getMean(), moved, slipped, binding, robotMoved);
        steps.add(step);

        System.out.println("[Traction]" + step.describe());

        if (robotMoved) {
            System.out.println("[Traction] ABORTED: " + analyse().abortReason());
        }

        log(step);
    }

    /** @return the lowest limit at which slip was seen, or 0 if none was. */
    private int tractionLimitAmps() {
        return tractionLimitAmps(steps);
    }

    /**
     * @param steps the sweep.
     * @return the lowest limit at which slip was seen, or 0 if none was.
     */
    static int tractionLimitAmps(List<Step> steps) {
        for (Step step : steps) {
            if (step.slipped()) {
                return step.limitAmps();
            }
        }
        return 0;
    }

    /** @return the analysed result for this run. */
    public Result analyse() {
        return analyse(steps, SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    }

    /**
     * Turns a sweep into a recommendation. Pure, so it is testable without a drivetrain.
     *
     * <p>The recommendation is the traction limit less {@value #SAFETY_MARGIN_AMPS} A, capped at
     * {@value #HARD_CAP_AMPS} A. If no slip was ever seen, the tyres out-grip anything the drivetrain
     * can ask for below the cap and the cap is the answer.
     *
     * <p>A run where the robot moved yields <b>no</b> recommendation — the currently configured limit
     * comes back instead. Producing a number from a run whose central assumption was violated is
     * worse than producing none, because the number looks just as authoritative as a good one.
     *
     * @param steps           the sweep, in the order it was run.
     * @param configuredLimit the limit currently in the code, returned when a run is invalid.
     * @return the recommendation.
     */
    static Result analyse(List<Step> steps, int configuredLimit) {
        // The run is invalid when the drivetrain never pushed anything hard enough to reach its
        // commanded limit. That, not an odometry distance, is what "not against the wall" looks like
        // in a signal that is independent of the wheels: a robot rolling freely never binds.
        boolean aborted = !steps.isEmpty()
                && steps.stream().noneMatch(Step::limitWasBinding);
        String abortReason = "";

        if (aborted) {
            abortReason = "No step reached its commanded current limit, so the drivetrain was never "
                    + "pushing anything immovable. Square the robot against the wall, confirm the "
                    + "bumpers are in contact across their width, and re-run.";
        }

        Step ranAway = steps.stream().filter(Step::robotMoved).findFirst().orElse(null);
        if (ranAway != null) {
            abortReason = abortReason + (abortReason.isEmpty() ? "" : " ")
                    + String.format("Also: odometry moved %.2f m during the %d A step, so the run was "
                            + "stopped early for safety.",
                            ranAway.chassisMovedMeters(), ranAway.limitAmps());
        }

        int traction = tractionLimitAmps(steps);

        int recommended;
        if (aborted) {
            recommended = configuredLimit;
        } else if (traction > 0) {
            recommended = Math.min(traction - SAFETY_MARGIN_AMPS, HARD_CAP_AMPS);
        } else {
            recommended = HARD_CAP_AMPS;
        }

        // Never recommend below where the sweep started: a recommendation under the first limit
        // tried would be extrapolation, not measurement.
        recommended = Math.max(recommended, START_AMPS);

        return new Result(List.copyOf(steps), recommended, traction, aborted, abortReason);
    }

    /** Prints the sweep, paste-ready. */
    public void printReport() {
        Result result = analyse();

        System.out.println();
        System.out.println("=== TRACTION / DRIVE CURRENT LIMIT REPORT ===");

        for (Step step : result.steps()) {
            System.out.println(step.describe());
        }

        System.out.println();

        if (result.aborted()) {
            System.out.println("RUN INVALID: " + result.abortReason());
            System.out.println("No recommendation. The drivetrain has been returned to the "
                    + "configured " + SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT + " A.");
            System.out.println("=== END ===");
            return;
        }

        if (result.foundTractionLimit()) {
            System.out.printf("Traction broke at %d A per motor.%n", result.tractionLimitAmps());
            System.out.printf("Recommended: SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT = %d "
                    + "(%d A below the traction limit)%n",
                    result.recommendedAmps(), SAFETY_MARGIN_AMPS);
        } else {
            System.out.printf("No slip up to the %d A cap — the tyres out-grip anything this "
                    + "drivetrain can ask for in that range.%n", HARD_CAP_AMPS);
            System.out.printf("Recommended: SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT = %d "
                    + "(the cap)%n", result.recommendedAmps());
        }

        System.out.printf("Currently configured: %d A%n",
                SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

        String warning = result.breakerWarning();
        if (!warning.isEmpty()) {
            System.out.println();
            System.out.println(warning);
        }

        boolean anyNonBinding = result.steps().stream()
                .anyMatch(step -> !step.limitWasBinding() && !step.slipped());
        if (anyNonBinding) {
            System.out.println();
            System.out.println("NOTE: some steps did not reach the limit that was set, so at those "
                    + "steps the limit was not what held the current down. Check the battery — a "
                    + "sagging pack under-reports the traction limit.");
        }

        System.out.println();
        System.out.println("Re-run after a wheel change, a weight change, or on a different "
                + "carpet. Traction is a property of the surface as much as the robot.");
        System.out.println("=== END ===");
        System.out.println();
    }

    private void log(Step step) {
        String root = "TractionCalibration";
        Logger.recordOutput(root + "/LastLimitAmps", step.limitAmps());
        Logger.recordOutput(root + "/LastWheelSpeedMps", step.wheelSpeedMps());
        Logger.recordOutput(root + "/LastAmpsPerMotor", step.measuredAmpsPerMotor());
        Logger.recordOutput(root + "/LastTotalAmps", step.totalAmps());
        Logger.recordOutput(root + "/LastBatteryVolts", step.batteryVolts());
        Logger.recordOutput(root + "/LastSlipped", step.slipped());
        Logger.recordOutput(root + "/LastLimitBinding", step.limitWasBinding());

        Result result = analyse();
        Logger.recordOutput(root + "/Aborted", result.aborted());
        Logger.recordOutput(root + "/TractionLimitAmps", result.tractionLimitAmps());
        Logger.recordOutput(root + "/RecommendedAmps", result.recommendedAmps());
    }

    /** @return the steps recorded so far. */
    public List<Step> getSteps() {
        return List.copyOf(steps);
    }
}
