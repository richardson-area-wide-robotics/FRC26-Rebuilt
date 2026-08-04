package frc.robot.common.components.diagnostics;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.rebuilt.subsystems.Feeder;
import frc.robot.rebuilt.subsystems.Intake;
import frc.robot.rebuilt.subsystems.Shooter;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * Guided routine that measures the current thresholds for game piece and jam detection.
 *
 * <p>Every value in {@code LoadConstants} is reasoned rather than measured. This is what replaces the
 * reasoning with numbers off the actual robot, one mechanism at a time.
 *
 * <p>Each mechanism runs three phases:
 *
 * <ol>
 *   <li><b>Empty</b> — the mechanism runs with nothing in it. Learns idle current, its noise, and
 *       free-running speed. Keep hands and game pieces clear.
 *   <li><b>Loaded</b> — keep feeding game pieces through for the whole phase. Gaps between pieces are
 *       fine and expected; {@link LoadCalibrator} works out which samples had a piece in them.
 *   <li><b>Obstructed</b> — hold the mechanism up so it cannot move product. Optional, and skipped
 *       for the shooter. This is what makes the jam threshold measured rather than inferred.
 * </ol>
 *
 * <p><b>Nothing here writes to source or to {@link CalibrationStore}.</b> It prints a report to be
 * read, sanity-checked and pasted by a human. These are per-mechanism physical measurements that
 * depend on which game piece was used and how worn the rollers are, so they do not meet the
 * auto-adopt rule — a value may only adopt itself if it is measurable independently of itself, and a
 * current threshold measured by feeding pieces through is not.
 *
 * <p><b>Requires an enabled robot and a human at the mechanism.</b> Run it from Test mode with the
 * robot on blocks. It is deliberately not part of the automated validation suite.
 */
public class LoadCalibrationRoutine {

    /** Seconds to let a mechanism reach steady state before any samples are taken. */
    private static final double SPIN_UP_SECONDS = 1.5;

    /**
     * Seconds of empty running.
     *
     * <p>4 seconds is 200 loops, comfortably past the 50 needed for a trustworthy noise figure, and
     * long enough to average over a few revolutions of an out-of-round roller.
     */
    private static final double EMPTY_SECONDS = 4.0;

    /**
     * Seconds of feeding pieces through.
     *
     * <p>Generous, because it is paced by a human picking up game pieces. Only the samples with a
     * piece present count towards the 25 needed, so a phase where pieces went through 20% of the time
     * still yields around 80 usable samples.
     */
    private static final double LOADED_SECONDS = 8.0;

    /**
     * Seconds of obstructed running.
     *
     * <p>Short on purpose. A brushless motor held near stall turns nearly all its input into heat in
     * the windings, and the SPARK's smart current limit caps the current but not the duration. Two
     * seconds is 100 samples, four times what the analysis needs.
     */
    private static final double JAM_SECONDS = 2.0;

    private final Intake intake;
    private final Feeder feeder;
    private final Shooter shooter;

    private final LoadCalibrator intakeCal = new LoadCalibrator("INTAKE");
    private final LoadCalibrator spindexerCal = new LoadCalibrator("SPINDEXER");
    private final LoadCalibrator feederCal = new LoadCalibrator("FEEDER");
    private final LoadCalibrator shooterCal = new LoadCalibrator("SHOOTER");

    /**
     * @param intake  Intake subsystem, or null to skip it.
     * @param feeder  Feeder subsystem, or null to skip it.
     * @param shooter Shooter subsystem, or null to skip it.
     */
    public LoadCalibrationRoutine(Intake intake, Feeder feeder, Shooter shooter) {
        this.intake = intake;
        this.feeder = feeder;
        this.shooter = shooter;
    }

    /**
     * Builds the three-phase sequence for one mechanism.
     *
     * <p>The mechanism is commanded to run for the whole sequence and stopped once at the end, so the
     * phases see one continuous run rather than three spin-ups. Restarting between phases would put a
     * spin-up transient into every population and inflate the noise figure the segmentation depends
     * on.
     *
     * @param cal      Accumulator for this mechanism.
     * @param owner    The subsystem being measured, declared as a requirement by every phase.
     * @param run      Starts the mechanism.
     * @param stop     Stops the mechanism.
     * @param amps     Supplies motor current.
     * @param speed    Supplies mechanism speed.
     * @param captureJam Whether to run the obstructed phase.
     * @return the sequence.
     */
    private Command calibrate(LoadCalibrator cal, Subsystem owner, Runnable run, Runnable stop,
            DoubleSupplier amps, DoubleSupplier speed, boolean captureJam) {

        // Every phase requires the mechanism it is measuring. Only the drivetrain has a default
        // command today, so nothing currently competes for these — but an operator holding the
        // intake button during a run would otherwise change what is being measured while the
        // routine carried on reporting as though it had not. With the requirement declared, a
        // button press interrupts the calibration visibly instead of corrupting it invisibly.
        Command empty = Commands.run(() -> {
            cal.addEmptySample(amps.getAsDouble(), speed.getAsDouble());
            cal.log();
        }, owner).withTimeout(EMPTY_SECONDS);

        Command loaded = Commands.run(() -> {
            cal.addLoadedSample(amps.getAsDouble(), speed.getAsDouble());
            cal.log();
        }, owner).withTimeout(LOADED_SECONDS);

        Command jam = Commands.run(() -> {
            cal.addJamSample(amps.getAsDouble(), speed.getAsDouble());
            cal.log();
        }, owner).withTimeout(JAM_SECONDS);

        List<Command> phases = new ArrayList<>();
        phases.add(announce(cal.getMechanism() + ": starting. Keep clear."));
        phases.add(Commands.runOnce(run, owner));
        phases.add(Commands.waitSeconds(SPIN_UP_SECONDS));
        phases.add(announce(cal.getMechanism() + ": EMPTY phase, " + EMPTY_SECONDS
                + "s. Nothing in the mechanism."));
        phases.add(empty);
        phases.add(announce(cal.getMechanism() + ": LOADED phase, " + LOADED_SECONDS
                + "s. Feed game pieces through continuously, starting NOW."));
        phases.add(loaded);

        if (captureJam) {
            phases.add(announce(cal.getMechanism() + ": OBSTRUCTED phase, " + JAM_SECONDS
                    + "s. Hold the mechanism so it cannot move product, starting NOW."));
            phases.add(jam);
        }

        phases.add(Commands.runOnce(stop, owner));
        phases.add(Commands.runOnce(() -> System.out.println(
                "  -> " + cal.recommend().describe())));

        return Commands.sequence(phases.toArray(new Command[0]))
                .withName("LoadCalibration/" + cal.getMechanism());
    }

    /** @return a command that prints one instruction line. */
    private static Command announce(String message) {
        return Commands.runOnce(() -> System.out.println("[LoadCalibration] " + message));
    }

    /** @return the intake roller routine, obstructed phase included. */
    public Command calibrateIntake() {
        if (intake == null) {
            return announce("INTAKE: skipped, no subsystem.");
        }
        return calibrate(intakeCal, intake, intake::intake, intake::stopRollers,
                intake::getRollerCurrent, intake::getRollerVelocity, true)
                .beforeStarting(Commands.runOnce(() -> intake.getRollerLoad().reset()));
    }

    /** @return the spindexer routine, obstructed phase included. */
    public Command calibrateSpindexer() {
        if (feeder == null) {
            return announce("SPINDEXER: skipped, no subsystem.");
        }
        return calibrate(spindexerCal, feeder, feeder::cycle, feeder::stopCycle,
                feeder::getSpindexerCurrent, feeder::getSpindexerVelocity, true)
                .beforeStarting(Commands.runOnce(() -> feeder.getSpindexerLoad().reset()));
    }

    /** @return the feeder routine, obstructed phase included. */
    public Command calibrateFeeder() {
        if (feeder == null) {
            return announce("FEEDER: skipped, no subsystem.");
        }
        return calibrate(feederCal, feeder, feeder::load, feeder::stopLoad,
                feeder::getFeederCurrent, feeder::getFeederVelocity, true)
                .beforeStarting(Commands.runOnce(() -> feeder.getFeederLoad().reset()));
    }

    /**
     * @return the flywheel routine, <b>without</b> an obstructed phase.
     *
     *     <p>Obstructing a flywheel by hand is how people lose fingers, and a wheel carrying that
     *     much momentum will not politely stall — it will either throw whatever is holding it or
     *     shear it. The jam threshold for the shooter is inferred from the loaded phase instead,
     *     which the report states plainly rather than implying it was measured.
     *
     *     <p>The flywheel also only spins when the hub interlock allows it, so this reports a skip
     *     rather than silently gathering a phase of zeros if the hub is inactive.
     */
    public Command calibrateShooter() {
        if (shooter == null) {
            return announce("SHOOTER: skipped, no subsystem.");
        }

        return Commands.either(
                calibrate(shooterCal, shooter, shooter::runShooter, shooter::stopShooter,
                        shooter::getShooterCurrent, shooter::getMeasuredRPM, false)
                        .beforeStarting(Commands.runOnce(() -> shooter.getFlywheelLoad().reset())),
                announce("SHOOTER: skipped, hub interlock inactive so the flywheel will not spin. "
                        + "Set the alliance and hub state, then re-run."),
                shooter::isHubActive);
    }

    /**
     * @return every mechanism in path order, upstream first.
     *
     *     <p>Intake, spindexer, feeder, shooter. Running them in the order pieces travel means a
     *     piece fed for one phase is roughly where it needs to be for the next, which matters when
     *     one person is doing both the feeding and the driver station.
     */
    public Command full() {
        return Commands.sequence(
                        announce("=== Load calibration starting. Robot on blocks, "
                                + "game pieces to hand. ==="),
                        calibrateIntake(),
                        calibrateSpindexer(),
                        calibrateFeeder(),
                        calibrateShooter(),
                        Commands.runOnce(this::printReport))
                .withName("LoadCalibration/Full");
    }

    /** Prints every recommendation gathered so far, paste-ready. */
    public void printReport() {
        System.out.println();
        System.out.println("=== LOAD CALIBRATION REPORT ===");
        System.out.println("Paste into RebuiltConstants.LoadConstants after sanity-checking.");
        System.out.println("Values marked NOT VIABLE must NOT be pasted — see the note against each.");
        System.out.println();

        for (LoadCalibrator cal : List.of(intakeCal, spindexerCal, feederCal, shooterCal)) {
            if (cal.getEmptySamples() == 0 && cal.getLoadedPhaseSamples() == 0) {
                System.out.println(cal.getMechanism() + ": not run.");
            } else {
                System.out.println(cal.recommend().describe());
            }
        }

        System.out.println();
        System.out.println("Reminder: these depend on the game piece used and on how worn the");
        System.out.println("mechanism is. Re-run after a roller change, a compression change, or");
        System.out.println("if piece counts start drifting during matches.");
        System.out.println("=== END ===");
        System.out.println();
    }

    /** @return the intake calibrator, for tests and dashboards. */
    public LoadCalibrator getIntakeCalibrator() {
        return intakeCal;
    }

    /** @return the spindexer calibrator. */
    public LoadCalibrator getSpindexerCalibrator() {
        return spindexerCal;
    }

    /** @return the feeder calibrator. */
    public LoadCalibrator getFeederCalibrator() {
        return feederCal;
    }

    /** @return the shooter calibrator. */
    public LoadCalibrator getShooterCalibrator() {
        return shooterCal;
    }
}
