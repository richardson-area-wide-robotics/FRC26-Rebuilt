package frc.robot.common.components.diagnostics;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.common.components.diagnostics.GatedStep.Assessment;
import frc.robot.common.components.diagnostics.GatedStep.Verdict;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Runs calibration steps under four buttons, and judges each result rather than printing it.
 *
 * <p>The loop per step is: read out the setup, wait for a button, and if that button was <b>RUN</b>,
 * measure and then assess. So the operator never has to read a console report and work out whether a
 * number is trustworthy &mdash; that judgement is in {@link GatedStep#assess()}, where it can be
 * tested.
 *
 * <h2>The four buttons</h2>
 *
 * <table>
 *   <caption>Buttons and what each one does</caption>
 *   <tr><th>Button</th><th>Does</th><th>Notes</th></tr>
 *   <tr><td><b>RUN</b></td><td>Measure this step now</td>
 *       <td>Also the re-run: pressing it again repeats the step, discarding the previous attempt</td></tr>
 *   <tr><td><b>NEXT</b></td><td>Accept the result and go forward</td>
 *       <td><b>Refused if the step has not been measured</b> &mdash; that is what SKIP is for</td></tr>
 *   <tr><td><b>PREVIOUS</b></td><td>Go back one step</td>
 *       <td>Its result is kept, so you can look at it again or re-run it</td></tr>
 *   <tr><td><b>SKIP</b></td><td>Go forward <em>without</em> a result</td>
 *       <td>Recorded as skipped, and named in the summary</td></tr>
 * </table>
 *
 * <p><b>RUN is deliberately also the re-run.</b> Starting a measurement and repeating one are the same
 * physical act, and a separate button for each is how somebody in a shop presses the wrong one.
 *
 * <p><b>NEXT and SKIP are deliberately different.</b> NEXT means "I have a result and I accept it";
 * SKIP means "I am moving on without one". Folding them together is what lets a session advance past
 * an unmeasured step and finish looking complete &mdash; so NEXT refuses when there is nothing to
 * accept, and says to press RUN or SKIP.
 *
 * <h2>Why this is index-driven rather than a fixed sequence</h2>
 *
 * <p>It used to be a flat {@code Commands.sequence} of steps with bounded retries, which cannot go
 * backwards: a sequence only moves one way and a step that has finished is gone. PREVIOUS therefore
 * required restructuring into a state machine &mdash; one command that reads the current index, does
 * exactly <em>one</em> thing, and repeats. Each iteration is a single button's worth of work, which is
 * also what makes the whole thing easy to follow.
 *
 * <h2>What it holds while running</h2>
 *
 * <p>It owns every subsystem it might touch for the entire run, which is what makes it safe to run
 * arbitrary inner commands. The consequence is that the drivetrain's default command is suspended, so
 * the robot could not otherwise be repositioned between steps &mdash; and repositioning is exactly
 * what the setup prompts ask for. So a <em>setup command</em> runs while waiting for a button, and the
 * container passes the ordinary teleop drive command for it. Positioning the robot therefore works
 * normally right up to the moment RUN is pressed.
 */
public class GuidedCalibration {

    /** Which button was pressed. */
    private enum Nav {
        RUN, NEXT, PREVIOUS, SKIP
    }

    private final BooleanSupplier runButton;
    private final BooleanSupplier nextButton;
    private final BooleanSupplier previousButton;
    private final BooleanSupplier skipButton;

    private final PressLatch runLatch = new PressLatch();
    private final PressLatch nextLatch = new PressLatch();
    private final PressLatch previousLatch = new PressLatch();
    private final PressLatch skipLatch = new PressLatch();

    private final Set<Subsystem> owned = new LinkedHashSet<>();
    private final List<GatedStep> steps = new ArrayList<>();
    private final Map<String, Assessment> results = new LinkedHashMap<>();
    private final Set<String> skipped = new LinkedHashSet<>();

    /** What runs while waiting for a button, so the robot can still be positioned. */
    private Supplier<Command> setupCommand = Commands::idle;

    private int index;
    private int announcedIndex = -1;
    private boolean finished;
    private Nav nav = Nav.RUN;

    /**
     * @param runButton      Reads true while RUN is held.
     * @param nextButton     Reads true while NEXT is held.
     * @param previousButton Reads true while PREVIOUS is held.
     * @param skipButton     Reads true while SKIP is held.
     * @param owned          Every subsystem any step might command. Held for the whole run.
     */
    public GuidedCalibration(BooleanSupplier runButton, BooleanSupplier nextButton,
            BooleanSupplier previousButton, BooleanSupplier skipButton, Subsystem... owned) {
        this.runButton = runButton;
        this.nextButton = nextButton;
        this.previousButton = previousButton;
        this.skipButton = skipButton;
        this.owned.addAll(Arrays.asList(owned));
    }

    /**
     * Sets what runs while waiting for a button.
     *
     * <p>Pass the ordinary teleop drive command. Without it the drivetrain sits inert for the whole
     * run, because this routine holds it &mdash; and the setup prompts ask the operator to reposition
     * the robot, which they then could not do.
     *
     * <p>A {@link Supplier} rather than a command, because a fresh instance is needed every time: the
     * same command object cannot be composed into two compositions.
     *
     * @param setupCommand Produces the command to run while waiting.
     * @return this, for chaining.
     */
    public GuidedCalibration whileWaiting(Supplier<Command> setupCommand) {
        this.setupCommand = setupCommand;
        return this;
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

    private static void say(String message) {
        System.out.println("[calib] " + message);
    }

    /** Prints the current step's setup and the button legend, but only on arriving at it. */
    private void announce() {
        if (announcedIndex == index) {
            return;
        }
        announcedIndex = index;

        GatedStep step = steps.get(index);
        Assessment previous = results.get(step.name());

        say("");
        say("--- step " + (index + 1) + " of " + steps.size() + ": " + step.name());
        say("SET UP: " + step.setupPrompt());

        if (previous != null) {
            say("    (already measured: " + previous.describe() + ")");
        }
        if (skipped.contains(step.name())) {
            say("    (previously skipped)");
        }

        say("    RUN = measure" + (previous != null ? " again" : "")
                + "   NEXT = accept and go on   PREVIOUS = back   SKIP = on without a result");
    }

    /**
     * Waits for any of the four buttons, recording which.
     *
     * <p>All four latches are polled every loop, so no button's press can be swallowed by another
     * being read first. Priority only matters when two are pressed on the same loop, which is a
     * fumble rather than an intent &mdash; RUN wins because it is the only non-destructive choice.
     *
     * @return the waiting command, which also runs the setup command so the robot can be positioned.
     */
    private Command awaitNav() {
        Command wait = Commands.sequence(
                Commands.runOnce(() -> {
                    runLatch.reset();
                    nextLatch.reset();
                    previousLatch.reset();
                    skipLatch.reset();
                }),
                Commands.waitUntil(() -> {
                    boolean run = runLatch.update(runButton.getAsBoolean());
                    boolean next = nextLatch.update(nextButton.getAsBoolean());
                    boolean previous = previousLatch.update(previousButton.getAsBoolean());
                    boolean skip = skipLatch.update(skipButton.getAsBoolean());

                    if (run) {
                        nav = Nav.RUN;
                    } else if (next) {
                        nav = Nav.NEXT;
                    } else if (previous) {
                        nav = Nav.PREVIOUS;
                    } else if (skip) {
                        nav = Nav.SKIP;
                    } else {
                        return false;
                    }
                    return true;
                }));

        // Raced against the setup command, so positioning the robot keeps working while waiting.
        return Commands.race(wait, setupCommand.get());
    }

    /** @return the measure-and-assess sequence for the current step. */
    private Command measureCurrent() {
        GatedStep step = steps.get(index);

        return Commands.sequence(
                Commands.runOnce(() -> {
                    // Reset before every attempt, including the first. A retry that appended to the
                    // previous attempt's population would carry a bad run forward invisibly, and the
                    // only sign would be a doubled sample count.
                    step.reset();
                    skipped.remove(step.name());
                    say("    Measuring. Hands clear.");
                }),
                step.measure(),
                Commands.runOnce(() -> {
                    Assessment assessment = step.assess();
                    results.put(step.name(), assessment);
                    say(" -> " + assessment.describe());

                    switch (assessment.verdict()) {
                        case PASS:
                            say("    Good. NEXT to go on, RUN for a second opinion.");
                            break;
                        case RETRY:
                            say("    Fix the above and press RUN. SKIP gives up on this step.");
                            break;
                        case FAIL:
                        default:
                            say("    Cannot be fixed on the robot. SKIP past it.");
                            break;
                    }

                    // Force the prompt to be reprinted, since the operator has just read a result and
                    // the button legend has scrolled away.
                    announcedIndex = -1;
                }));
    }

    /** Applies a navigation button. */
    private void navigate() {
        GatedStep step = steps.get(index);

        switch (nav) {
            case NEXT:
                if (results.get(step.name()) == null) {
                    // The distinction that stops a session finishing while looking complete.
                    say("    NOT MEASURED yet, so there is nothing to accept. Press RUN to measure "
                            + "it, or SKIP to move on without it.");
                    return;
                }
                if (results.get(step.name()).verdict() != Verdict.PASS) {
                    say("    Going on with a result that did not pass. Do not paste this one.");
                }
                advance();
                break;

            case PREVIOUS:
                if (index == 0) {
                    say("    Already at the first step.");
                    return;
                }
                index--;
                announcedIndex = -1;
                break;

            case SKIP:
                skipped.add(step.name());
                results.remove(step.name());
                say("    SKIPPED " + step.name() + ".");
                advance();
                break;

            case RUN:
            default:
                break;
        }
    }

    private void advance() {
        if (index >= steps.size() - 1) {
            finished = true;
            return;
        }
        index++;
        announcedIndex = -1;
    }

    /** Prints what may be adopted and what still needs re-gathering. */
    private void printSummary() {
        say("");
        say("================ CALIBRATION SUMMARY ================");

        List<String> good = new ArrayList<>();
        List<String> regather = new ArrayList<>();

        for (GatedStep step : steps) {
            Assessment assessment = results.get(step.name());
            if (assessment != null && assessment.verdict() == Verdict.PASS) {
                good.add(step.name() + " -- " + assessment.detail());
            } else if (skipped.contains(step.name())) {
                regather.add(step.name() + " -- skipped");
            } else if (assessment == null) {
                regather.add(step.name() + " -- never run");
            } else {
                regather.add(step.name() + " -- " + assessment.detail());
            }
        }

        say("USABLE (" + good.size() + "/" + steps.size() + "):");
        for (String line : good) {
            say("  + " + line);
        }

        if (!regather.isEmpty()) {
            say("NEEDS RE-GATHERING:");
            for (String line : regather) {
                say("  - " + line);
            }
            // Stated as a rule rather than a suggestion. A half-calibrated robot carrying numbers from
            // the passing half is harder to reason about than an uncalibrated one, because the
            // constants stop telling you which are measured.
            say("Do not paste values for anything on that list.");
        } else {
            say("Everything usable. Run tools/apply_sysid.py for the drive feedforward.");
        }
    }

    /**
     * @return the whole guided routine.
     *
     *     <p>One iteration does exactly one button's worth of work: print the prompt if the step has
     *     just changed, wait for a button, then either measure or navigate. Repeating that is the whole
     *     state machine, and it is what makes PREVIOUS possible at all.
     */
    public Command full() {
        if (steps.isEmpty()) {
            return Commands.runOnce(() -> say("No steps configured."));
        }

        Command iteration = Commands.sequence(
                Commands.runOnce(this::announce),
                awaitNav(),
                Commands.either(
                        Commands.defer(this::measureCurrent, owned),
                        Commands.runOnce(this::navigate),
                        () -> nav == Nav.RUN));

        return Commands.sequence(
                Commands.runOnce(() -> {
                    index = 0;
                    announcedIndex = -1;
                    finished = false;
                    results.clear();
                    skipped.clear();

                    say("================================================================");
                    say("GUIDED CALIBRATION -- " + steps.size() + " steps, four buttons.");
                    say("Each step judges its own data and says what to change.");
                    say("================================================================");
                }),

                // Deferred, because the step it builds depends on an index that changes as it runs --
                // a command built up front would be frozen on step one for ever.
                Commands.defer(() -> iteration, owned)
                        .repeatedly()
                        .until(() -> finished),

                Commands.runOnce(this::printSummary))
                .withName("GuidedCalibration");
    }
}
