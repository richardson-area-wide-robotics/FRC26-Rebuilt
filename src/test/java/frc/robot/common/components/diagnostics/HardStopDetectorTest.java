package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import frc.robot.common.components.diagnostics.HardStopDetector.End;
import frc.robot.common.components.diagnostics.HardStopDetector.State;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Tests for telling a physical end stop apart from a game piece.
 *
 * <p>The whole point is that current cannot make this distinction — both cases are "current up, speed
 * down". So every test here is really asking the same question: does the code look at whether
 * <b>position is frozen or creeping</b>, or has it fallen back on current?
 *
 * <p>The consequence of getting it wrong is not just a wrong indicator. A ball mistaken for a hard
 * stop would let the encoder be re-zeroed against the ball's position, and the soft limits are
 * expressed in encoder units — so the error would propagate into the thing meant to stop the arm
 * hitting the frame.
 */
class HardStopDetectorTest {

    /** Position range counting as frozen. */
    private static final double FROZEN_BAND = 0.05;

    /** Current above which the motor is pushing. */
    private static final double PUSHING_AMPS = 8.0;

    /** Loops the condition must hold. */
    private static final int SUSTAIN = 12;

    /** Enough output to count as commanded. */
    private static final double DEPLOY_OUT = 0.2;
    private static final double STOW_OUT = -0.25;

    private HardStopDetector detector;

    @BeforeEach
    void setUp() {
        detector = new HardStopDetector("Test/Arm", FROZEN_BAND, PUSHING_AMPS, SUSTAIN);
    }

    /** Drives the arm onto a stop: position pinned, current high. */
    private void pushIntoStop(double position, double output, int loops) {
        for (int i = 0; i < loops; i++) {
            // A tiny amount of encoder noise, well inside the frozen band.
            detector.update(position + (i % 2) * 0.002, 20.0, output);
        }
    }

    @Nested
    @DisplayName("a real hard stop")
    class RealStop {

        @Test
        @DisplayName("is detected once position has been frozen long enough")
        void detected() {
            pushIntoStop(11.0, DEPLOY_OUT, SUSTAIN * 3);

            assertEquals(State.AT_HARD_STOP, detector.getState());
            assertTrue(detector.isAtHardStop());
            assertFalse(detector.isObstructed());
        }

        @Test
        @DisplayName("is not declared early, before the sustain window")
        void notDeclaredEarly() {
            // Enough samples to fill the window but not to satisfy the sustain requirement. A stop
            // declared this fast could not have ruled out a ball that yields slowly.
            pushIntoStop(11.0, DEPLOY_OUT, SUSTAIN + 2);

            assertNotEquals(State.AT_HARD_STOP, detector.getState(),
                "a stop must not be declared before the sustain window has elapsed");
        }

        @Test
        @DisplayName("learns which end it was, from the direction of push")
        void learnsPerEnd() {
            pushIntoStop(11.0, DEPLOY_OUT, SUSTAIN * 3);
            assertEquals(11.0, detector.getLearnedStop(End.HIGH), 0.01);
            assertTrue(Double.isNaN(detector.getLearnedStop(End.LOW)),
                "pushing one way says nothing about the other end");

            detector.reset();
            pushIntoStop(0.0, STOW_OUT, SUSTAIN * 3);
            assertEquals(0.0, detector.getLearnedStop(End.LOW), 0.01);
        }

        @Test
        @DisplayName("measures the travel between the two ends")
        void measuresTravel() {
            pushIntoStop(0.2, STOW_OUT, SUSTAIN * 3);
            detector.reset();
            pushIntoStop(10.4, DEPLOY_OUT, SUSTAIN * 3);

            // This is the number that validates DEPLOY_POSITION_ROTATIONS and the soft limits, which
            // have been hand-chosen and unchecked since the first review.
            assertEquals(10.2, detector.getMeasuredTravel(), 0.05);
        }
    }

    @Nested
    @DisplayName("a game piece in the way")
    class Ball {

        @Test
        @DisplayName("is reported as an obstruction, not a stop")
        void notMistakenForAStop() {
            // The signature that fools a current-only detector: high current, barely moving. But a
            // ball yields, so position creeps past the frozen band over the window.
            for (int i = 0; i < SUSTAIN * 3; i++) {
                detector.update(5.0 + i * 0.02, 22.0, DEPLOY_OUT);
            }

            assertEquals(State.OBSTRUCTED, detector.getState());
            assertFalse(detector.isAtHardStop(),
                "a ball must never be reported as the end of travel");
        }

        @Test
        @DisplayName("never contributes a learned stop position")
        void neverLearnsFromABall() {
            for (int i = 0; i < SUSTAIN * 3; i++) {
                detector.update(5.0 + i * 0.02, 22.0, DEPLOY_OUT);
            }

            // If it did, the encoder could be re-zeroed against a ball, and the soft limits are in
            // the same units — the error would propagate into the guard against hitting the frame.
            assertTrue(Double.isNaN(detector.getLearnedStop(End.HIGH)));
            assertEquals(0, detector.getStopHits(End.HIGH));
        }

        @Test
        @DisplayName("a ball that finally stops dead does read as a stop, and honestly so")
        void aTrulyImmovableBallIsIndistinguishable() {
            // Worth being explicit about the limit of this method. A ball wedged so hard it cannot
            // move is, mechanically, a hard stop — there is no signal left to tell them apart. Hence
            // the runbook's instruction to calibrate with no game pieces loaded.
            pushIntoStop(5.0, DEPLOY_OUT, SUSTAIN * 3);
            assertTrue(detector.isAtHardStop());
        }
    }

    @Nested
    @DisplayName("a soft limit")
    class SoftLimit {

        @Test
        @DisplayName("is distinguished from a hard stop by current falling, not rising")
        void distinguishedFromHardStop() {
            // The SPARK enforces a soft limit by cutting output, so position freezes with almost no
            // current. A detector watching only for "stopped" would call this a hard stop.
            for (int i = 0; i < SUSTAIN * 3; i++) {
                detector.update(11.0, 0.5, DEPLOY_OUT);
            }

            assertEquals(State.AT_SOFT_LIMIT, detector.getState());
            assertFalse(detector.isAtHardStop());
        }

        @Test
        @DisplayName("is never learned as a position reference")
        void neverLearnedAsAReference() {
            for (int i = 0; i < SUSTAIN * 3; i++) {
                detector.update(11.0, 0.5, DEPLOY_OUT);
            }

            // Re-zeroing against a soft limit would be circular: the limit is itself defined in
            // encoder units, so the error would be self-reinforcing once wrong.
            assertTrue(Double.isNaN(detector.getLearnedStop(End.HIGH)));
        }
    }

    @Nested
    @DisplayName("encoder drift")
    class Drift {

        @Test
        @DisplayName("is the gap between where the stop is and where it was assumed to be")
        void measuresTheBootAssumptionError() {
            // The arm boots part-way, so the encoder reads 0 there. Later it reaches the real stowed
            // stop and the encoder says -1.4: the constructor's assumption was out by that much, and
            // the soft limits were out by the same amount.
            pushIntoStop(-1.4, STOW_OUT, SUSTAIN * 3);

            assertEquals(-1.4, detector.getEncoderDrift(End.LOW, 0.0), 0.01);
        }

        @Test
        @DisplayName("is unavailable unless the arm is actually against the stop")
        void unavailableAwayFromTheStop() {
            for (int i = 0; i < 5; i++) {
                detector.update(5.0 + i, 3.0, DEPLOY_OUT);
            }

            assertTrue(Double.isNaN(detector.getEncoderDrift(End.LOW, 0.0)),
                "drift is only meaningful against a confirmed physical reference");
        }
    }

    @Nested
    @DisplayName("idle and moving")
    class Ordinary {

        @Test
        @DisplayName("a stationary arm with no command is idle, not stopped")
        void idleIsNotAStop() {
            for (int i = 0; i < SUSTAIN * 3; i++) {
                detector.update(0.0, 0.2, 0.0);
            }

            assertEquals(State.IDLE, detector.getState());
            assertFalse(detector.isAtHardStop());
        }

        @Test
        @DisplayName("the arm's idle hold bias still counts as commanded")
        void holdBiasCounts() {
            // stopDeploy() applies -0.03 to keep the arm against its stow stop. That is genuinely
            // commanded, and treating it as idle would mean the stow stop is never seen at all —
            // which is the one the encoder most needs to be re-zeroed against.
            pushIntoStop(0.0, -0.03, SUSTAIN * 3);
            assertEquals(State.AT_HARD_STOP, detector.getState());
        }

        @Test
        @DisplayName("a normally moving arm reports moving")
        void movingReportsMoving() {
            for (int i = 0; i < SUSTAIN * 3; i++) {
                detector.update(i * 0.3, 6.0, DEPLOY_OUT);
            }

            assertEquals(State.MOVING, detector.getState());
        }
    }
}
