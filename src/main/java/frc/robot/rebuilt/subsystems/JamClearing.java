package frc.robot.rebuilt.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

/**
 * Automates the jostling that currently gets done by hand.
 *
 * <p>When a game piece jams, the fix is to pump the mechanism back and forth until it frees — the
 * intake up and down, and sometimes the spindexer and feeder too. That works because a jam is
 * usually a piece wedged in a stable position, and breaking the symmetry lets gravity and the
 * rollers finish the job. It does not need to be done by a human; it needs to be done quickly and
 * consistently, which is what a robot is better at.
 *
 * <p>Four properties make this safe to run automatically:
 *
 * <ul>
 *   <li><b>Bounded.</b> Every routine has an attempt limit and gives up, reporting failure. The
 *       existing {@code Intake.jiggleItALittleCommand()} is an unbounded
 *       {@code repeatingSequence} — it would jostle until interrupted, which is fine for a button
 *       and unacceptable for something triggered by a sensor.</li>
 *   <li><b>Self-terminating.</b> Each attempt re-checks whether the jam has cleared and exits
 *       immediately if so, rather than running a fixed routine to completion.</li>
 *   <li><b>Escalating.</b> Later attempts pump harder and longer. Most jams clear on the gentlest
 *       action, and starting gentle avoids throwing pieces or straining a mechanism for a jam that
 *       a nudge would have fixed.</li>
 *   <li><b>Loud.</b> Every attempt is logged and printed. A mechanism that moves without the driver
 *       asking must be traceable afterwards, and a robot that jams eleven times a match is telling
 *       you something a mechanism change should fix rather than software papering over.</li>
 * </ul>
 */
public final class JamClearing {

    /** How many attempts before giving up and reporting. */
    public static final int MAX_ATTEMPTS = 3;

    /** Base duration of one direction of a pump, in seconds. */
    private static final double BASE_PUMP_SECONDS = 0.25;

    /** Each attempt lengthens the pump by this factor. */
    private static final double ESCALATION_FACTOR = 1.4;

    /** Pause after a pump so the piece can settle before the jam is re-checked. */
    private static final double SETTLE_SECONDS = 0.15;

    private JamClearing() {
    }

    /**
     * Clears an intake jam by pumping the deploy arm, the motion done by hand today.
     *
     * @param intake    Intake to jostle.
     * @param stillJammed Supplies whether the jam persists; checked between attempts.
     * @return a bounded, self-terminating command.
     */
    public static Command intakeJostle(Intake intake, BooleanSupplier stillJammed) {
        Command sequence = Commands.runOnce(() ->
                announce("intake", "pumping the deploy arm up and down"));

        for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
            final int number = attempt;
            double duration = BASE_PUMP_SECONDS * Math.pow(ESCALATION_FACTOR, attempt - 1);

            sequence = sequence.andThen(
                    Commands.either(
                            Commands.none(),
                            onePump(
                                    "intake", number, duration,
                                    Commands.run(intake::manualReverseDeploy, intake),
                                    Commands.run(intake::manualDeploy, intake),
                                    Commands.runOnce(intake::stopDeploy, intake)),
                            // Skip the remaining attempts entirely once it has cleared.
                            () -> !stillJammed.getAsBoolean()));
        }

        return sequence
                .andThen(Commands.runOnce(() -> report("intake", stillJammed.getAsBoolean())))
                .withName("ClearIntakeJam");
    }

    /**
     * Clears a spindexer jam by reversing and re-advancing it.
     *
     * @param feeder      Feeder subsystem, which owns the spindexer.
     * @param stillJammed Supplies whether the jam persists.
     * @return a bounded, self-terminating command.
     */
    public static Command spindexerJostle(Feeder feeder, BooleanSupplier stillJammed) {
        Command sequence = Commands.runOnce(() ->
                announce("spindexer", "reversing and re-advancing"));

        for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
            final int number = attempt;
            double duration = BASE_PUMP_SECONDS * Math.pow(ESCALATION_FACTOR, attempt - 1);

            sequence = sequence.andThen(
                    Commands.either(
                            Commands.none(),
                            onePump(
                                    "spindexer", number, duration,
                                    Commands.run(feeder::reverseCycle, feeder),
                                    Commands.run(feeder::cycle, feeder),
                                    Commands.runOnce(feeder::holdCycle, feeder)),
                            () -> !stillJammed.getAsBoolean()));
        }

        return sequence
                .andThen(Commands.runOnce(() -> report("spindexer", stillJammed.getAsBoolean())))
                .withName("ClearSpindexerJam");
    }

    /**
     * Clears a feeder jam by reversing and re-advancing it.
     *
     * <p>The feeder is the vertical run up to the flywheel — the mechanism referred to as the tower.
     * If a separate tower motor is ever added, this needs its own routine rather than reusing this
     * one.
     *
     * @param feeder      Feeder to jostle.
     * @param stillJammed Supplies whether the jam persists.
     * @return a bounded, self-terminating command.
     */
    public static Command feederJostle(Feeder feeder, BooleanSupplier stillJammed) {
        Command sequence = Commands.runOnce(() ->
                announce("feeder", "reversing and re-advancing"));

        for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
            final int number = attempt;
            double duration = BASE_PUMP_SECONDS * Math.pow(ESCALATION_FACTOR, attempt - 1);

            sequence = sequence.andThen(
                    Commands.either(
                            Commands.none(),
                            onePump(
                                    "feeder", number, duration,
                                    Commands.run(feeder::reverseLoad, feeder),
                                    Commands.run(feeder::load, feeder),
                                    Commands.runOnce(feeder::stopLoad, feeder)),
                            () -> !stillJammed.getAsBoolean()));
        }

        return sequence
                .andThen(Commands.runOnce(() -> report("feeder", stillJammed.getAsBoolean())))
                .withName("ClearFeederJam");
    }

    /**
     * Clears the whole ball path at once, working from the shooter end backwards.
     *
     * <p>Order matters. Clearing the feeder first gives the pieces behind it somewhere to go; doing
     * the intake first would push more pieces into an already-blocked path.
     *
     * @param intake         Intake to jostle.
     * @param feeder         Feeder and spindexer to jostle.
     * @param anyStillJammed Supplies whether anything in the path is still jammed.
     * @return a bounded, self-terminating command.
     */
    public static Command clearWholePath(
            Intake intake, Feeder feeder, BooleanSupplier anyStillJammed) {
        return Commands.sequence(
                        Commands.runOnce(() -> announce("path",
                                "clearing from the shooter end backwards")),
                        feederJostle(feeder, anyStillJammed),
                        spindexerJostle(feeder, anyStillJammed),
                        intakeJostle(intake, anyStillJammed))
                .withName("ClearBallPath");
    }

    /**
     * One reverse-then-forward pump.
     *
     * @param mechanism Name for logging.
     * @param attempt   Attempt number, for the escalation log.
     * @param seconds   How long each direction runs.
     * @param reverse   Command that drives the mechanism backwards.
     * @param forward   Command that drives it forwards.
     * @param rest      Command that returns it to its resting demand.
     */
    private static Command onePump(String mechanism, int attempt, double seconds,
            Command reverse, Command forward, Command rest) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    System.out.printf("[jam] %s attempt %d of %d, %.2f s each way%n",
                            mechanism, attempt, MAX_ATTEMPTS, seconds);
                    Logger.recordOutput("Jam/" + mechanism + "/Attempt", attempt);
                }),
                reverse.withTimeout(seconds),
                forward.withTimeout(seconds),
                rest,
                Commands.waitSeconds(SETTLE_SECONDS));
    }

    private static void announce(String mechanism, String what) {
        System.out.println("[jam] " + mechanism + " jam detected — " + what);
        Logger.recordOutput("Jam/" + mechanism + "/Clearing", true);
    }

    private static void report(String mechanism, boolean stillJammed) {
        Logger.recordOutput("Jam/" + mechanism + "/Clearing", false);
        Logger.recordOutput("Jam/" + mechanism + "/ClearedSuccessfully", !stillJammed);

        if (stillJammed) {
            System.out.println("[jam] " + mechanism + " STILL JAMMED after " + MAX_ATTEMPTS
                    + " attempts — needs a human. Repeated jams here are a mechanism problem, not "
                    + "something software should keep papering over.");
        } else {
            System.out.println("[jam] " + mechanism + " cleared");
        }
    }
}
