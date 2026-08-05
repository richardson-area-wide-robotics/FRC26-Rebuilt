package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/** Sequencing for the button that paces the hand-motion calibration. */
class PressLatchTest {

    /** @return advances counted while the button is held for the given number of loops. */
    private static int advancesWhileHeld(PressLatch latch, int loops) {
        int advances = 0;
        for (int i = 0; i < loops; i++) {
            if (latch.update(true)) {
                advances++;
            }
        }
        return advances;
    }

    @Test
    @DisplayName("A press held for twenty loops advances exactly once")
    void heldPressAdvancesOnce() {
        // The failure this prevents is silent, which is why it is worth its own test. A step that
        // advanced on the raw button would advance on every loop the button was down — twenty or so for
        // a normal press — running the whole routine to the end on one press while printing a report for
        // each step it never measured. The log would look like a completed calibration.
        PressLatch latch = new PressLatch();
        latch.update(false);

        assertEquals(1, advancesWhileHeld(latch, 20),
            "a single physical press must be a single advance no matter how long it is held");
    }

    @Test
    @DisplayName("A button already held at the start waits for a release")
    void alreadyHeldWaitsForRelease() {
        // An operator with a hand on the controller, or one who started the routine with the same
        // button they are about to advance with. Firing straight away would skip step one before its
        // instruction had been read.
        PressLatch latch = new PressLatch();

        assertEquals(0, advancesWhileHeld(latch, 10),
            "a press that was already in progress is not a new instruction");

        latch.update(false);
        assertTrue(latch.update(true), "after a release, the next press counts");
    }

    @Test
    @DisplayName("Each press advances one step, across several presses")
    void severalPressesAdvanceSeveralSteps() {
        PressLatch latch = new PressLatch();

        for (int press = 0; press < 4; press++) {
            latch.update(false);
            advancesWhileHeld(latch, 15);
        }

        assertEquals(4, latch.getAdvances(), "four presses, four advances");
    }

    @Test
    @DisplayName("reset() makes the tail of an ongoing press stop counting")
    void resetRequiresAFreshPress() {
        // Steps reset the latch as they begin. Without this, a step that ended while the button was
        // still down could accept the remainder of that same press as its own advance — so one press
        // would advance two steps, and only the second would be skipped. Harder to notice than
        // skipping all of them.
        PressLatch latch = new PressLatch();
        latch.update(false);
        assertTrue(latch.update(true), "the first press advances");

        latch.reset();

        assertFalse(latch.update(true),
            "still inside the same press, so it must not advance the next step");

        latch.update(false);
        assertTrue(latch.update(true), "a genuinely new press does advance it");
    }
}
