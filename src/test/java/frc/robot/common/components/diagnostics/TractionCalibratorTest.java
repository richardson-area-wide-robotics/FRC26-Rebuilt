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

    /**
     * Builds a step where the robot rolled instead of pushing: wheels turning, current nowhere near
     * the commanded limit, odometry running on.
     */
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

        /**
         * A sweep in which no step ever reached its commanded limit.
         *
         * <p>That is what not being against the wall actually looks like in a signal independent of
         * the wheels: a drivetrain with nothing to push against rolls freely and never binds. It used
         * to be judged on odometry distance instead, which could not work — see
         * {@link #odometryDistanceIsNotAValidityTest}.
         */
        private Result result() {
            List<Step> steps = new ArrayList<>();
            steps.add(droveAway(20));
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
        @DisplayName("says what to do about it, not just that it failed")
        void explainsWhy() {
            String reason = result().abortReason();

            assertTrue(reason.contains("current limit"), reason);
            assertTrue(reason.contains("wall"), reason);
        }

        @Test
        @DisplayName("Odometry distance alone must never invalidate a run")
        void odometryDistanceIsNotAValidityTest() {
            // The defect this pins was arithmetic, and it made the whole routine useless.
            //
            // Slip used to be "wheels turning AND the robot staying put", with staying put read from
            // drive.getPose() — the pose estimator, which is wheel-integrated odometry whenever no
            // AprilTag is in view, which is always in a shop. Spinning wheels therefore advance it.
            // Sustained wheel speed above the slip threshold necessarily moved the estimate further
            // than the static tolerance, so "the robot stayed put" was false exactly when slip
            // happened. The two halves of the test excluded one another, the step that genuinely
            // broke traction aborted the run as "not against the wall", and re-squaring the robot
            // produced the same message every time.
            //
            // So: a step with metres of odometry travel, but binding at its limit and wheels well
            // past the slip threshold, has to be usable. This is that step.
            Step slippingHard = new Step(45, 2.40, 43.0, 172.0, 12.0, 3.20, true, true, false);

            List<Step> steps = new ArrayList<>();
            steps.add(gripped(40));
            steps.add(slippingHard);
            Result result = TractionCalibrator.analyse(steps, CONFIGURED_LIMIT);

            assertFalse(result.aborted(),
                    "the drivetrain was binding at its limit, so the run is valid however far "
                            + "odometry thinks the robot went");
            assertTrue(result.foundTractionLimit(),
                    "this is the whole point of the routine: it has to be able to find a limit");
            assertEquals(45, result.tractionLimitAmps());
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
            assertTrue(droveAway(40).describe().contains("STOPPED"),
                    "a runaway is now a safety stop, not a verdict on the data");

            Step notBinding = new Step(70, 0.01, 42.0, 168.0, 10.8, 0.01, false, false, false);
            assertTrue(notBinding.describe().contains("not binding"), notBinding.describe());
        }

        @Test
        @DisplayName("an empty sweep yields no recommendation and does not throw")
        void emptySweep() {
            Result result = TractionCalibrator.analyse(List.of(), CONFIGURED_LIMIT);

            // Not "aborted": there is nothing to abort. The abort message names a specific thing the
            // operator did wrong, so printing it for a run that never started is a wrong instruction.
            assertFalse(result.aborted());
            assertFalse(result.foundTractionLimit());
            assertEquals(TractionCalibrator.HARD_CAP_AMPS, result.recommendedAmps());
        }
    }
}
