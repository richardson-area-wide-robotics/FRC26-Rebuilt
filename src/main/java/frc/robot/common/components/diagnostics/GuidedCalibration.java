package frc.robot.common.components.diagnostics;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.components.diagnostics.GatedStep.Assessment;
import frc.robot.common.components.diagnostics.GatedStep.Verdict;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * Runs calibration steps on two buttons, and decides for itself whether each result is good enough.
 *
 * <p>The loop per step is: read out the setup, wait for <b>READY</b>, measure, judge the data, then
 * either move on or say what to change and offer the step again. So the operator never has to read a
 * console report and work out whether the number is trustworthy &mdash; that judgement is in
 * {@link GatedStep#assess()}, where it can be tested.
 *
 * <h2>Two buttons, and what they mean depends on where you are</h2>
 *
 * <table>
 *   <caption>Button meanings</caption>
 *   <tr><th>State</th><th>READY</th><th>NEXT</th></tr>
 *   <tr><td>Setup prompt shown</td><td>Start measuring</td><td>&mdash;</td></tr>
 *   <tr><td>Result was PASS</td><td>Re-run it anyway</td><td>Move to the next step</td></tr>
 *   <tr><td>Result was RETRY</td><td><b>Try again</b></td><td>Give up on this step, move on</td></tr>
 * </table>
 *
 * <p>Giving up is on the same two buttons on purpose. A step that cannot be made to pass must not be
 * able to trap the session &mdash; and a third button is one more thing to explain while somebody is
 * holding a robot.
 *
 * <h2>Why attempts are bounded rather than unbounded</h2>
 *
 * <p>Five attempts, then the step reports as unresolved and the sequence continues. Partly because
 * data that will not come good in five tries is telling you something the routine cannot fix, and
 * partly because a bounded sequence of distinct commands avoids the framework edge cases a
 * self-restarting one has &mdash; a repeating composition re-announces its prompt on the loop it is
 * being cancelled, which reads as the step starting again just as it finishes.
 */
public class GuidedCalibration {

    /** Attempts allowed per step before it is recorded as unresolved. */
    private static final int MAX_ATTEMPTS = 5;

    private final BooleanSupplier readyButton;
    private final BooleanSupplier nextButton;

    private final PressLatch readyLatch = new PressLatch();
    private final PressLatch nextLatch = new PressLatch();

    private final List<GatedStep> steps = new ArrayList<>();
    private final Map<String, Assessment> results = new LinkedHashMap<>();

    /** True once the current step should stop being re-attempted. */
    private boolean stepSettled;

    /**
     * @param readyButton Reads true while the READY button is held.
     * @param nextButton  Reads true while the NEXT button is held.
     */
    public GuidedCalibration(BooleanSupplier readyButton, BooleanSupplier nextButton) {
        this.readyButton = readyButton;
        this.nextButton = nextButton;
    }

    /**
     * Adds a step. Order is the order they run in.
     *
     * @param step The step.
     * @return this, for chaining.
     */
    public GuidedCalibration add(GatedStep step) {
        steps.add(step);
        return this;
    }

    private static Command say(String message) {
        return Commands.runOnce(() -> System.out.println("[calib] " + message));
    }

    /** @return a command that finishes on the next fresh press of READY. */
    private Command awaitReady() {
        return Commands.sequence(
                Commands.runOnce(readyLatch::reset),
                Commands.waitUntil(() -> readyLatch.update(readyButton.getAsBoolean())));
    }

    /**
     * Waits for either button and records which.
     *
     * @param advanced Single-element flag set true when NEXT was the one pressed.
     * @return the waiting command.
     */
    private Command awaitChoice(boolean[] advanced) {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    readyLatch.reset();
                    nextLatch.reset();
                    advanced[0] = false;
                }),
                Commands.waitUntil(() -> {
                    // Both are polled every loop. The latches are independent, so reading one cannot
                    // swallow the other's press.
                    boolean ready = readyLatch.update(readyButton.getAsBoolean());
                    boolean next = nextLatch.update(nextButton.getAsBoolean());
                    if (next) {
                        advanced[0] = true;
                    }
                    return ready || next;
                }));
    }

    /**
     * Builds one attempt at a step.
     *
     * @param step    The step.
     * @param attempt 1-based attempt number, for the prompt.
     * @return the attempt, which does nothing if the step has already settled.
     */
    private Command attempt(GatedStep step, int attempt) {
        boolean[] advanced = new boolean[1];

        Command body = Commands.sequence(
                say(""),
                say("--- " + step.name()
                        + (attempt > 1 ? "  (attempt " + attempt + " of " + MAX_ATTEMPTS + ")" : "")),
                say("SET UP: " + step.setupPrompt()),
                say("        Press READY when it is set up."),
                awaitReady(),

                // Reset before every attempt, including the first. A retry that appended to the
                // previous attempt's data would carry the bad run forward invisibly.
                Commands.runOnce(step::reset),
                say("        Measuring. Hands clear."),
                step.measure(),

                Commands.runOnce(() -> {
                    Assessment assessment = step.assess();
                    results.put(step.name(), assessment);
                    System.out.println("[calib]  -> " + assessment.describe());

                    if (assessment.verdict() == Verdict.PASS) {
                        System.out.println("[calib]        Press NEXT to continue, or READY to re-run.");
                    } else if (assessment.verdict() == Verdict.RETRY) {
                        System.out.println("[calib]        Fix the above, then press READY to try "
                                + "again. NEXT skips this step.");
                    } else {
                        System.out.println("[calib]        Cannot be fixed on the robot. Press NEXT.");
                    }
                }),

                awaitChoice(advanced),

                Commands.runOnce(() -> {
                    Assessment assessment = results.get(step.name());
                    boolean pass = assessment != null && assessment.verdict() == Verdict.PASS;
                    boolean unrecoverable = assessment != null
                            && assessment.verdict() == Verdict.FAIL;

                    // Settled when the operator chose to move on, or when re-running cannot help.
                    //
                    // A PASS is deliberately NOT enough on its own: pressing READY on a step that
                    // already passed re-runs it, which is a legitimate thing to want when a number
                    // looks surprising and a second opinion is cheap. The operator decides when a
                    // step is done; the assessment only decides what advice comes with it.
                    stepSettled = advanced[0] || unrecoverable;

                    if (pass && !advanced[0]) {
                        System.out.println("[calib]        Re-running a step that already passed.");
                    }
                }));

        return body.onlyIf(() -> !stepSettled);
    }

    /** @return the full sequence for one step, bounded at {@link #MAX_ATTEMPTS} attempts. */
    private Command step(GatedStep step) {
        List<Command> attempts = new ArrayList<>();
        attempts.add(Commands.runOnce(() -> stepSettled = false));

        // Distinct command instances: WPILib forbids composing the same command object twice, and a
        // self-repeating composition would have the re-announce problem described on the class.
        for (int i = 1; i <= MAX_ATTEMPTS; i++) {
            attempts.add(attempt(step, i));
        }

        attempts.add(Commands.runOnce(() -> {
            Assessment assessment = results.get(step.name());
            if (assessment == null || assessment.verdict() == Verdict.RETRY) {
                System.out.println("[calib]  " + step.name()
                        + ": UNRESOLVED after " + MAX_ATTEMPTS + " attempts. Moving on.");
            }
        }));

        return Commands.sequence(attempts.toArray(new Command[0]));
    }

    /** Prints what may be adopted and what still needs re-gathering. */
    private void printSummary() {
        System.out.println("");
        System.out.println("[calib] ================ CALIBRATION SUMMARY ================");

        List<String> good = new ArrayList<>();
        List<String> regather = new ArrayList<>();

        for (GatedStep step : steps) {
            Assessment assessment = results.get(step.name());
            if (assessment != null && assessment.verdict() == Verdict.PASS) {
                good.add(step.name());
            } else {
                regather.add(step.name() + " -- "
                        + (assessment == null ? "never run" : assessment.detail()));
            }
        }

        System.out.println("[calib] USABLE (" + good.size() + "/" + steps.size() + "):");
        for (String name : good) {
            System.out.println("[calib]   + " + name);
        }

        if (!regather.isEmpty()) {
            System.out.println("[calib] NEEDS RE-GATHERING:");
            for (String line : regather) {
                System.out.println("[calib]   - " + line);
            }
            // Stated as a rule rather than a suggestion. A partially-calibrated robot with numbers
            // pasted from the passing half is harder to reason about than an uncalibrated one, because
            // the constants no longer tell you which are measured.
            System.out.println("[calib] Do not paste values for anything on that list.");
        } else {
            System.out.println("[calib] Everything usable. Run tools/apply_sysid.py for the "
                    + "drive feedforward.");
        }
    }

    /** @return the whole guided sequence. */
    public Command full() {
        List<Command> all = new ArrayList<>();
        all.add(say("================================================================"));
        all.add(say("GUIDED CALIBRATION -- " + steps.size() + " steps."));
        all.add(say("READY starts a measurement. NEXT moves on once it has passed."));
        all.add(say("Each step judges its own data and will tell you to re-gather."));
        all.add(say("================================================================"));

        for (GatedStep step : steps) {
            all.add(step(step));
        }

        all.add(Commands.runOnce(this::printSummary));
        return Commands.sequence(all.toArray(new Command[0])).withName("GuidedCalibration");
    }
}
