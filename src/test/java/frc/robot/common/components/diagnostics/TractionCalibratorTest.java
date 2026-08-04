package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.common.components.diagnostics.TractionCalibrator.Result;
import frc.robot.common.components.diagnostics.TractionCalibrator.Step;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Tests for the drive current limit analysis.
 *
 * <p>The sweep itself needs a robot against a wall, but turning a sweep into a number does not, and
 * the number is the part that ends up in the code. These cover the cases that matter: a clean sweep,
 * a sweep that never breaks traction, and a sweep taken with the robot not against the wall — where
 * the correct output is no recommendation at all.
 */
class TractionCalibratorTest {

    /** The limit currently in the code, returned when a run is invalid. */
    private static final int CONFIGURED_LIMIT = 50;

    /** Builds a gripping step at the given limit. */
    private static Step gripped(int amps) {
        return new Step(amps, 0.02, amps * 0.95, amps * 0.95 * 4, 12.4, 0.01,
                false, true, false);
    }

    /** Builds a slipping step at the given limit. */
    private static Step slipped(int amps) {
        return new Step(amps, 1.80, amps * 0.95, amps * 0.95 * 4, 12.1, 0.02,
                true, true, false);
    }

    /** Builds a step where the robot drove away instead of pushing. */
    private static Step droveAway(int amps) {
        return new Step(amps, 1.20, amps * 0.60, amps * 0.60 * 4, 12.3, 1.40,
                false, false, true);
    }

    /** A sweep that grips up to {@code slipAt} and slips there. */
    private static List<Step> sweepSlippingAt(int slipAt) {
        List<Step> steps = new ArrayList<>();
        for (int amps = 20; amps <= slipAt; amps += 5) {
            steps.add(amps == slipAt ? slipped(amps) : gripped(amps));
        }
        return steps;
    }

    @Nested
    @DisplayName("a sweep that finds the traction limit")
    class FoundTheLimit {

        @Test
        @DisplayName("recommends below the limit where the wheels broke loose")
        void recommendsBelowTheTractionLimit() {
            Result result = TractionCalibrator.analyse(sweepSlippingAt(55), CONFIGURED_LIMIT);

            assertTrue(result.foundTractionLimit());
            assertEquals(55, result.tractionLimitAmps());
            assertTrue(result.recommendedAmps() < 55,
                    "recommending at or above the traction limit means slipping under load, got "
                            + result.recommendedAmps());
            assertEquals(50, result.recommendedAmps());
        }

        @Test
        @DisplayName("takes the first slip, not a later one")
        void takesTheFirstSlip() {
            // Once the wheels are loose they stay loose, so later steps also slip. The limit is where
            // it first happened.
            List<Step> steps = new ArrayList<>(sweepSlippingAt(40));
            steps.add(slipped(45));
            steps.add(slipped(50));

            assertEquals(40, TractionCalibrator.analyse(steps, CONFIGURED_LIMIT)
                    .tractionLimitAmps());
        }

        @Test
        @DisplayName("never recommends above the 80 A cap")
        void respectsTheCap() {
            // A drivetrain that only slips beyond the cap: the cap wins, as asked.
            List<Step> steps = new ArrayList<>();
            for (int amps = 20; amps <= 80; amps += 5) {
                steps.add(gripped(amps));
            }

            Result result = TractionCalibrator.analyse(steps, CONFIGURED_LIMIT);

            assertFalse(result.foundTractionLimit());
            assertEquals(TractionCalibrator.HARD_CAP_AMPS, result.recommendedAmps());
        }

        @Test
        @DisplayName("never recommends below the first limit tried")
        void neverExtrapolatesBelowTheSweep() {
            // Slip on the very first step. Backing off 5 A from there would recommend 15 A, which no
            // step ever tested — so the floor is where the sweep started.
            Result result = TractionCalibrator.analyse(List.of(slipped(20)), CONFIGURED_LIMIT);

            assertEquals(20, result.recommendedAmps(),
                    "a recommendation below the lowest limit tried is extrapolation");
        }
    }

    @Nested
    @DisplayName("a sweep where the robot was not against the wall")
    class NotAgainstTheWall {

        private Result result() {
            List<Step> steps = new ArrayList<>();
            steps.add(gripped(20));
            steps.add(droveAway(25));
            return TractionCalibrator.analyse(steps, CONFIGURED_LIMIT);
        }

        @Test
        @DisplayName("is reported as aborted")
        void isAborted() {
            assertTrue(result().aborted());
        }

        @Test
        @DisplayName("hands back the configured limit rather than inventing one")
        void keepsTheConfiguredLimit() {
            assertEquals(CONFIGURED_LIMIT, result().recommendedAmps(),
                    "a number from an invalid run looks just as authoritative as a good one, so "
                            + "there must not be one");
        }

        @Test
        @DisplayName("says how far the robot moved, so the operator knows what went wrong")
        void explainsWhy() {
            String reason = result().abortReason();

            assertTrue(reason.contains("1.40"), reason);
            assertTrue(reason.contains("25"), reason);
            assertFalse(reason.isEmpty());
        }

        @Test
        @DisplayName("wheel rotation while driving away is not counted as slip")
        void drivingIsNotSlip() {
            // droveAway has the wheels turning at 1.2 m/s. Without the chassis check that would be
            // the most convincing slip in the sweep.
            assertEquals(0, result().tractionLimitAmps(),
                    "turning wheels only mean slip when the robot is not moving");
        }
    }

    @Nested
    @DisplayName("the main breaker warning")
    class BreakerWarning {

        @Test
        @DisplayName("fires when four motors at the recommended limit exceed 120 A")
        void warnsAboveTheBreaker() {
            Result result = TractionCalibrator.analyse(sweepSlippingAt(45), CONFIGURED_LIMIT);

            // 40 A per motor is 160 A across four.
            assertEquals(40, result.recommendedAmps());
            assertFalse(result.breakerWarning().isEmpty(),
                    "160 A against a 120 A breaker is worth saying out loud");
            assertTrue(result.breakerWarning().contains("160"), result.breakerWarning());
        }

        @Test
        @DisplayName("stays quiet when the total is within the breaker")
        void quietBelowTheBreaker() {
            // 25 A per motor is 100 A across four, inside the breaker.
            Result result = TractionCalibrator.analyse(sweepSlippingAt(30), CONFIGURED_LIMIT);

            assertEquals(25, result.recommendedAmps());
            assertTrue(result.breakerWarning().isEmpty());
        }
    }

    @Nested
    @DisplayName("step reporting")
    class StepReporting {

        @Test
        @DisplayName("distinguishes gripped, slipped, invalid and non-binding")
        void describesEachOutcome() {
            assertTrue(gripped(40).describe().contains("gripped"));
            assertTrue(slipped(40).describe().contains("SLIPPED"));
            assertTrue(droveAway(40).describe().contains("INVALID"));

            Step notBinding = new Step(70, 0.01, 42.0, 168.0, 10.8, 0.01, false, false, false);
            assertTrue(notBinding.describe().contains("not binding"), notBinding.describe());
        }

        @Test
        @DisplayName("an empty sweep yields no recommendation and does not throw")
        void emptySweep() {
            Result result = TractionCalibrator.analyse(List.of(), CONFIGURED_LIMIT);

            assertFalse(result.aborted());
            assertFalse(result.foundTractionLimit());
            assertEquals(TractionCalibrator.HARD_CAP_AMPS, result.recommendedAmps());
        }
    }
}
