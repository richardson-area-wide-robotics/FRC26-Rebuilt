package frc.robot.common.components.diagnostics;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.rebuilt.RebuiltConstants.IntakeConstants;
import frc.robot.rebuilt.subsystems.Feeder;
import frc.robot.rebuilt.subsystems.Intake;
import frc.robot.rebuilt.subsystems.Shooter;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * The first calibration step: every mechanism moved <b>by hand</b>, motors unpowered, to establish
 * which way is positive and where the arm's stops are.
 *
 * <p>Runs before anything is driven under power, and that ordering is the point. A sign error found
 * here costs fifteen seconds; the same error found by the drive characterisation produces a negative
 * gain, and found by the arm's profile produces an arm driven into its own hard stop at full command.
 * Both of those look like mechanical faults, so they cost an afternoon on the mechanism before anyone
 * suspects a sign.
 *
 * <p>It also produces the arm's travel and soft limits from its two physical stops. That measurement
 * is <b>ground truth for the powered travel test</b>, which turns {@link DeployTravelCalibrator} from a
 * discovery into a check: it is far better to already know the span before driving the arm at a stop
 * and watching the current to find out.
 *
 * <h2>Pacing</h2>
 *
 * <p>Every step waits for a button rather than a timer, because the operator has both hands on the
 * robot and no idea when a four-second window started. {@link PressLatch} makes one press advance
 * exactly one step; without it a single press would tear through the whole routine, printing a report
 * for each skipped step.
 *
 * <h2>What this needs of the robot</h2>
 *
 * <ul>
 *   <li><b>Enabled</b>, because commands do not run otherwise — but nothing is ever commanded to move.
 *   <li>The drivetrain is declared as a requirement so its default command is suspended. Without that,
 *       the steering modules keep servoing to their last reference and fight the hand turning them.
 *   <li>Brake is restored from {@code finallyDo}, so an interrupted run cannot leave the arm free to
 *       fall or the drivetrain free to roll.
 * </ul>
 *
 * <p><b>Take the arm's weight before its steps and do not let go.</b> It is not balanced, so it falls
 * in coast.
 */
public class HandMotionRoutine {

    /**
     * Loop period used to integrate velocity into travel.
     *
     * <p>Only the mechanisms with no position reading need this, and only to decide whether enough
     * movement happened to be a measurement. The direction is in the sign, which no timing error can
     * change, so a nominal period is honest here rather than approximate.
     */
    private static final double LOOP_SECONDS = 0.02;

    private final SwerveDriveSubsystem drive;
    private final Intake intake;
    private final Feeder feeder;
    private final Shooter shooter;
    private final BooleanSupplier nextButton;

    private final List<HandMotionCalibrator.Result> results = new ArrayList<>();

    /**
     * @param drive      Drivetrain, or null to skip its steps.
     * @param intake     Intake, or null to skip its steps.
     * @param feeder     Feeder, or null to skip its steps.
     * @param shooter    Shooter, or null to skip its steps.
     * @param nextButton Reads true while the operator's advance button is held. Any button on the
     *                   controller or a dashboard boolean; the latch does not care which.
     */
    public HandMotionRoutine(SwerveDriveSubsystem drive, Intake intake, Feeder feeder,
            Shooter shooter, BooleanSupplier nextButton) {
        this.drive = drive;
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
        this.nextButton = nextButton;
    }

    /** @return a command that prints one line. */
    private static Command announce(String message) {
        return Commands.runOnce(() -> System.out.println("[HandMotion] " + message));
    }

    /**
     * Builds one polarity step: sample until the button, then report.
     *
     * @param mechanism   Human name.
     * @param instruction What the operator should do, phrased as the positive direction.
     * @param signal      Position reading for this mechanism.
     * @param byVelocity  True when the signal is a velocity rather than a position.
     * @param owner       Subsystem to hold for the step.
     * @return the step.
     */
    private Command polarityStep(String mechanism, String instruction, DoubleSupplier signal,
            boolean byVelocity, Subsystem owner) {

        HandMotionCalibrator.Motion motion = new HandMotionCalibrator.Motion(mechanism, instruction);
        PressLatch latch = new PressLatch();

        Command sample = Commands.run(() -> {
            if (byVelocity) {
                motion.addVelocity(signal.getAsDouble(), LOOP_SECONDS);
            } else {
                motion.addPosition(signal.getAsDouble());
            }
        }, owner).until(() -> latch.update(nextButton.getAsBoolean()));

        return Commands.sequence(
                announce(""),
                announce(mechanism + " -- " + instruction),
                announce("            Then press NEXT. (Release the button first if you are holding it.)"),
                // Reset immediately before sampling, not at construction: the routine is built once
                // and the previous step's release must not arm this one.
                Commands.runOnce(latch::reset),
                sample,
                Commands.runOnce(() -> {
                    HandMotionCalibrator.Result result = motion.result();
                    results.add(result);
                    System.out.println("      " + result.describe());
                }));
    }

    /**
     * Builds the arm's two-stop travel measurement.
     *
     * <p>Separate from the polarity steps because it reads a position at a moment rather than
     * accumulating motion, and because what it produces is a pair of soft limits rather than a sign.
     * The sign falls out of it for free, which is why the arm needs no polarity step of its own.
     *
     * @return the step, or a skip notice when there is no intake.
     */
    public Command measureArmTravel() {
        if (intake == null) {
            return announce("ARM TRAVEL: skipped, no intake subsystem.");
        }

        PressLatch stowLatch = new PressLatch();
        PressLatch deployLatch = new PressLatch();
        double[] captured = new double[2];

        return Commands.sequence(
                announce(""),
                announce("ARM TRAVEL -- take the arm's weight NOW and do not let go. It is in coast."),
                announce("            Move it to the FULLY STOWED stop, until it is against steel."),
                announce("            Then press NEXT."),
                Commands.runOnce(stowLatch::reset),
                Commands.run(() -> {
                }, intake).until(() -> stowLatch.update(nextButton.getAsBoolean())),
                Commands.runOnce(() -> {
                    captured[0] = intake.getDeployPosition();
                    System.out.println("      stowed stop at " + captured[0] + " rotations");
                }),

                announce("            Now move it to the FULLY DEPLOYED stop, against steel."),
                announce("            Then press NEXT."),
                Commands.runOnce(deployLatch::reset),
                Commands.run(() -> {
                }, intake).until(() -> deployLatch.update(nextButton.getAsBoolean())),
                Commands.runOnce(() -> {
                    captured[1] = intake.getDeployPosition();
                    System.out.println("      deployed stop at " + captured[1] + " rotations");

                    HandMotionCalibrator.ArmTravel travel =
                            new HandMotionCalibrator.ArmTravel(captured[0], captured[1]);
                    System.out.println("      " + travel.describe(
                            IntakeConstants.STOW_POSITION_ROTATIONS,
                            IntakeConstants.DEPLOY_POSITION_ROTATIONS));
                }));
    }

    /** @return the four drive-wheel polarity steps, one per module. */
    public Command driveSteps() {
        if (drive == null) {
            return announce("DRIVE: skipped, no drivetrain.");
        }

        String[] names = {"FRONT-LEFT", "FRONT-RIGHT", "REAR-LEFT", "REAR-RIGHT"};
        List<Command> steps = new ArrayList<>();

        for (int i = 0; i < names.length; i++) {
            final int module = i;
            steps.add(polarityStep(names[module] + " DRIVE WHEEL",
                    "roll this wheel the way it turns when the robot drives FORWARD",
                    () -> drive.getModuleDrivePositions()[module], false, drive));
        }

        for (int i = 0; i < names.length; i++) {
            final int module = i;
            steps.add(polarityStep(names[module] + " STEERING",
                    "turn this module COUNTER-CLOCKWISE seen from above (the positive yaw direction)",
                    () -> drive.getRawTurnPositionsRadians()[module], false, drive));
        }

        return Commands.sequence(steps.toArray(new Command[0]));
    }

    /** @return polarity steps for every mechanism that is not the drivetrain or the arm. */
    public Command mechanismSteps() {
        List<Command> steps = new ArrayList<>();

        if (intake != null) {
            steps.add(polarityStep("INTAKE ROLLERS",
                    "turn the rollers the way they move a ball INTO the robot",
                    intake::getRollerVelocitySigned, true, intake));
        }
        if (feeder != null) {
            steps.add(polarityStep("SPINDEXER",
                    "turn the spindexer the way it moves balls TOWARD the feeder",
                    feeder::getSpindexerVelocity, true, feeder));
            steps.add(polarityStep("FEEDER",
                    "turn the feeder the way it moves a ball UP toward the shooter",
                    feeder::getFeederVelocity, true, feeder));
        }
        if (shooter != null) {
            steps.add(polarityStep("SHOOTER FLYWHEEL",
                    "spin the flywheel the way it throws a ball OUT",
                    () -> shooter.getMeasuredRPM() / 60.0, true, shooter));
        }

        if (steps.isEmpty()) {
            return announce("MECHANISMS: skipped, no subsystems.");
        }
        return Commands.sequence(steps.toArray(new Command[0]));
    }

    /** Puts every mechanism that needs it into coast, and stops anything still driving. */
    private void enterCoast() {
        if (drive != null) {
            drive.setCoastForHandCalibration(true);
        }
        if (intake != null) {
            // The rollers are already configured to coast, so only the arm needs the change. Stopping
            // them is still worth doing: coast does not stop a motor that is running.
            intake.stopRollers();
            intake.setDeployCoastForHandCalibration(true);
        }
        if (feeder != null) {
            feeder.setCoastForHandCalibration(true);
        }
        if (shooter != null) {
            // Already coast, like the rollers — a flywheel always is. It only needs to be stopped.
            shooter.stopShooter();
        }
    }

    /** Restores brake everywhere. Must run even when the routine is interrupted. */
    private void restoreBrake() {
        if (drive != null) {
            drive.setCoastForHandCalibration(false);
        }
        if (intake != null) {
            intake.setDeployCoastForHandCalibration(false);
        }
        if (feeder != null) {
            feeder.setCoastForHandCalibration(false);
        }
        System.out.println("[HandMotion] Brake restored. The arm is held again.");
    }

    /** Prints the summary, listing what has to change before any powered test. */
    private void printSummary() {
        System.out.println("");
        System.out.println("[HandMotion] ================ SUMMARY ================");

        List<HandMotionCalibrator.Result> inverted = new ArrayList<>();
        List<HandMotionCalibrator.Result> unmeasured = new ArrayList<>();

        for (HandMotionCalibrator.Result result : results) {
            if (result.polarity() == HandMotionCalibrator.Polarity.INVERTED) {
                inverted.add(result);
            } else if (!result.isConclusive()) {
                unmeasured.add(result);
            }
        }

        if (inverted.isEmpty() && unmeasured.isEmpty()) {
            System.out.println("[HandMotion] All " + results.size()
                    + " mechanisms agree with the code. Cleared for powered tests.");
            return;
        }

        for (HandMotionCalibrator.Result result : inverted) {
            System.out.println("[HandMotion] INVERTED: " + result.mechanism());
        }
        for (HandMotionCalibrator.Result result : unmeasured) {
            System.out.println("[HandMotion] NOT MEASURED: " + result.mechanism()
                    + " (" + result.polarity() + ")");
        }

        // Stated as a blocker rather than a note. An inverted mechanism is the one condition under
        // which every powered test downstream produces a confident wrong answer.
        System.out.println("[HandMotion] Fix the inversions above, then re-run this step.");
        System.out.println("[HandMotion] Do NOT run powered tests until every mechanism agrees.");
    }

    /**
     * @return the whole routine: coast, every step, then brake restored regardless of how it ended.
     *
     *     <p>{@code finallyDo} rather than a final command in the sequence, because a sequence that is
     *     interrupted never reaches its last command — and the thing that would be skipped is the one
     *     that stops the arm falling.
     */
    public Command full() {
        return Commands.sequence(
                announce("================================================================"),
                announce("HAND-MOTION CALIBRATION -- motors unpowered. Nothing here moves itself."),
                announce("Take the arm's weight before its steps; in coast it falls."),
                announce("Advance each step with the NEXT button. One press, one step."),
                announce("================================================================"),
                Commands.runOnce(this::enterCoast),
                driveSteps(),
                mechanismSteps(),
                measureArmTravel(),
                Commands.runOnce(this::printSummary))
                .finallyDo(interrupted -> restoreBrake())
                .withName("HandMotionCalibration");
    }
}
