package frc.robot.common.components.diagnostics;

/**
 * Turns a held button into exactly one advance event.
 *
 * <p>Guided calibration steps are paced by a human pressing a button, and a button read at 50 Hz is
 * pressed for something like ten to thirty loops. A step that advanced on {@code pressed == true}
 * would therefore advance ten to thirty times, tearing through the whole routine on one press and
 * leaving every step after the first unmeasured — while still printing a report for each, because
 * nothing would have told it the step was skipped. That is the failure this exists to prevent, and
 * it is worth a class of its own because it is silent: the log looks like a completed run.
 *
 * <p><b>A release is required before a press counts.</b> That covers the case where the operator is
 * already holding the button when the routine starts — mid-press, or leaning on the controller —
 * which would otherwise fire an immediate advance before the first instruction had even been read.
 *
 * <p>Pure logic and framework-free on purpose, so the sequencing can be tested without a HAL, a
 * joystick or a robot.
 */
public class PressLatch {

    /**
     * Whether a press would currently be accepted.
     *
     * <p>Starts false, which is what makes a button that is already down at construction wait for a
     * release rather than firing straight away.
     */
    private boolean armed;

    /** Total accepted presses, for reporting how far a routine actually got. */
    private int advances;

    /**
     * Folds in one reading of the button.
     *
     * @param pressed Whether the button is down this loop.
     * @return true on the loop a press is accepted, and on no other loop of that press.
     */
    public boolean update(boolean pressed) {
        if (!pressed) {
            armed = true;
            return false;
        }

        if (!armed) {
            // Still inside a press that has already been counted, or a press that began before this
            // latch was watching. Either way it is not a new instruction from the operator.
            return false;
        }

        armed = false;
        advances++;
        return true;
    }

    /** @return how many presses have been accepted. */
    public int getAdvances() {
        return advances;
    }

    /**
     * Forgets the current press, so the next advance needs a fresh one.
     *
     * <p>Called between steps. Without it, a latch that ended a step still armed could accept the
     * tail of the same physical press as the next step's advance.
     */
    public void reset() {
        armed = false;
    }
}
