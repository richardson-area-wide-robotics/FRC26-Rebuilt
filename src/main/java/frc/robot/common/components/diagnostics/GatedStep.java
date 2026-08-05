package frc.robot.common.components.diagnostics;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * One calibration step that can judge its own data.
 *
 * <p>The point of the interface is the third method. Every calibration in this package already knows
 * whether its result is usable &mdash; a regression knows its R&sup2; and sample count, a traction
 * sweep knows whether the drivetrain ever bound, a load calibration knows whether its two populations
 * separated. That knowledge was previously buried in printed prose, so acting on it meant a human
 * reading a console and deciding. {@link #assess()} lifts it into something a routine can branch on,
 * which is what lets the sequence say <em>re-gather this</em> instead of carrying on.
 *
 * <p>Implementations are thin adapters over the existing calibrators. None of the measurement logic
 * belongs here.
 */
public interface GatedStep {

    /** How the data came out. */
    enum Verdict {
        /** Good enough to keep and move on. */
        PASS,
        /** Not usable, but re-running could fix it. The operator gets told what to change. */
        RETRY,
        /** Not usable and re-running will not help. Something has to change off-robot first. */
        FAIL
    }

    /**
     * A verdict and the reason for it.
     *
     * @param verdict What to do next.
     * @param detail  Why, in the operator's terms. For {@link Verdict#RETRY} this must say what to
     *                <em>change</em> before re-running, because "re-gather" on its own invites the
     *                same run a second time and the same verdict a second time.
     */
    record Assessment(Verdict verdict, String detail) {

        /** @return a passing assessment. */
        public static Assessment pass(String detail) {
            return new Assessment(Verdict.PASS, detail);
        }

        /** @return a retryable assessment. {@code detail} should name the fix. */
        public static Assessment retry(String detail) {
            return new Assessment(Verdict.RETRY, detail);
        }

        /** @return an unrecoverable assessment. */
        public static Assessment fail(String detail) {
            return new Assessment(Verdict.FAIL, detail);
        }

        /** @return a line for the console. */
        public String describe() {
            return verdict + " -- " + detail;
        }
    }

    /** @return short human name, e.g. {@code "TRACTION LIMIT"}. */
    String name();

    /**
     * @return what the operator must physically do before this step can run.
     *
     *     <p>Read out before the READY button is waited on, and read out again on every retry, since
     *     a retry usually means the setup was the problem.
     */
    String setupPrompt();

    /** @return the measurement itself. A fresh command each call, so a retry can re-run it. */
    Command measure();

    /** @return the verdict on the data just gathered. */
    Assessment assess();

    /**
     * Discards the previous attempt's data.
     *
     * <p>Called before every attempt, including the first. Without it a retry appends to the previous
     * run's population rather than replacing it, so a bad first attempt permanently contaminates
     * every attempt after it &mdash; and the only visible sign is a doubled sample count.
     */
    default void reset() {
    }
}
