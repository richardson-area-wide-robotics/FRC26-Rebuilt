package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.common.components.diagnostics.HandMotionCalibrator.ArmTravel;
import frc.robot.common.components.diagnostics.HandMotionCalibrator.Motion;
import frc.robot.common.components.diagnostics.HandMotionCalibrator.Polarity;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/** Polarity and arm-travel analysis from mechanisms moved by hand. */
class HandMotionCalibratorTest {

    @Nested
    @DisplayName("Polarity")
    class PolarityTests {

        @Test
        @DisplayName("A clean push the requested way reads AGREES")
        void cleanForwardAgrees() {
            Motion motion = new Motion("FRONT-LEFT DRIVE", "roll forward");
            for (double p = 0; p <= 3.0; p += 0.1) {
                motion.addPosition(p);
            }

            assertEquals(Polarity.AGREES, motion.result().polarity());
        }

        @Test
        @DisplayName("A clean push the requested way, encoder falling, reads INVERTED")
        void cleanBackwardIsInverted() {
            Motion motion = new Motion("FRONT-LEFT DRIVE", "roll forward");
            for (double p = 0; p >= -3.0; p -= 0.1) {
                motion.addPosition(p);
            }

            HandMotionCalibrator.Result result = motion.result();
            assertEquals(Polarity.INVERTED, result.polarity());
            assertTrue(result.describe().contains("INVERTED"),
                "the report has to name the problem, not just the numbers");
            assertTrue(result.describe().contains("powered test"),
                "and it has to say not to run powered tests until it is fixed");
        }

        @Test
        @DisplayName("The first reading is a reference, not a jump from zero")
        void firstReadingDoesNotAccumulate() {
            // An arm sitting at 8 rotations when the step begins would otherwise record an 8-rotation
            // move before the operator touched it — enough on its own to satisfy the travel gate and
            // decide the polarity from where the encoder happened to be.
            Motion motion = new Motion("ARM", "deploy");
            motion.addPosition(8.0);

            assertEquals(0.0, motion.getNet(), 1e-9);
            assertEquals(0.0, motion.getTravel(), 1e-9);
            assertEquals(Polarity.NO_MOTION, motion.result().polarity());
        }

        @Test
        @DisplayName("Not moving it at all reads NO_MOTION, not a direction")
        void noMotionIsNotADirection() {
            Motion motion = new Motion("FEEDER", "move a ball up");
            for (int i = 0; i < 50; i++) {
                motion.addPosition(0.001 * (i % 2));   // noise, no movement
            }

            assertEquals(Polarity.NO_MOTION, motion.result().polarity());
            assertFalse(motion.result().isConclusive());
        }

        @Test
        @DisplayName("Wobbling it back and forth reads AMBIGUOUS, not whichever way it ended")
        void wobbleIsAmbiguous() {
            // The case that would otherwise produce a confident wrong answer: a hand that pushed a
            // wheel back and forth and happened to finish slightly forward. Net travel alone cannot
            // tell that from a deliberate push, which is why absolute travel is tracked too.
            Motion motion = new Motion("SPINDEXER", "move balls toward the feeder");
            for (int cycle = 0; cycle < 6; cycle++) {
                motion.addPosition(0.0);
                motion.addPosition(1.0);
            }
            motion.addPosition(0.2);

            HandMotionCalibrator.Result result = motion.result();
            assertEquals(Polarity.AMBIGUOUS, result.polarity(),
                "net " + result.net() + " out of " + result.travel() + " travel is not a direction");
            assertTrue(result.travel() > HandMotionCalibrator.MIN_TRAVEL_ROTATIONS,
                "this has to be distinct from NO_MOTION — plenty moved, it just cancelled out");
        }

        @Test
        @DisplayName("A barely-moved mechanism reads NO_MOTION rather than AMBIGUOUS")
        void tinyWobblePrefersNoMotion() {
            // Ordering matters in the report, not just the enum. AMBIGUOUS tells the operator to move it
            // more steadily; NO_MOTION tells them it may not be reading at all. A tiny jiggle satisfies
            // neither directional test, and sending someone to steady their hand when the encoder is
            // dead is the wrong instruction.
            Motion motion = new Motion("SHOOTER", "spin the way it throws out");
            motion.addPosition(0.0);
            motion.addPosition(0.05);
            motion.addPosition(0.0);

            assertEquals(Polarity.NO_MOTION, motion.result().polarity());
        }

        @Test
        @DisplayName("Velocity integration finds the same direction as position")
        void velocityWorksForMechanismsWithNoPosition() {
            // Flywheel and rollers report velocity only. A slow careful hand gives a small
            // instantaneous value, so what has to accumulate is the integral.
            Motion motion = new Motion("SHOOTER FLYWHEEL", "spin the way it throws out");
            for (int i = 0; i < 100; i++) {
                motion.addVelocity(-2.0, 0.02);
            }

            HandMotionCalibrator.Result result = motion.result();
            assertEquals(Polarity.INVERTED, result.polarity());
            assertEquals(-4.0, result.net(), 1e-6, "100 samples at -2/s over 20 ms each");
        }
    }

    @Nested
    @DisplayName("Arm travel")
    class ArmTravelTests {

        @Test
        @DisplayName("Soft limits are held back from the stops, whichever way the arm counts")
        void limitsAreInsetFromBothStops() {
            // A soft limit sitting exactly on a hard stop is a soft limit reached by hitting steel, so
            // the inset is what makes it a limit rather than a record of the crash. It has to be inset
            // in the right direction for both sign conventions, which is where a signum is easy to get
            // backwards — and backwards means each limit sits OUTSIDE its stop, disabling both.
            ArmTravel rising = new ArmTravel(0.0, 10.0);
            assertEquals(0.25, rising.recommendedStowLimit(), 1e-9);
            assertEquals(9.75, rising.recommendedDeployLimit(), 1e-9);

            ArmTravel falling = new ArmTravel(0.0, -10.0);
            assertEquals(-0.25, falling.recommendedStowLimit(), 1e-9);
            assertEquals(-9.75, falling.recommendedDeployLimit(), 1e-9);

            assertTrue(rising.deployIsPositive());
            assertFalse(falling.deployIsPositive());
        }

        @Test
        @DisplayName("Two readings from the same place are reported as not measured")
        void bothStopsAtTheSamePlaceIsNotAMeasurement() {
            // What happens when the operator presses NEXT twice without moving the arm. Reporting a
            // travel of 0.02 rotations and a pair of soft limits derived from it would bake a broken arm
            // into constants that look measured.
            ArmTravel nothing = new ArmTravel(4.0, 4.02);

            assertFalse(nothing.isUsable());
            assertTrue(nothing.describe(0.0, 10.0).contains("NOT MEASURED"));
        }

        @Test
        @DisplayName("An arm that travels the opposite way to constants is flagged as a sign mismatch")
        void signMismatchIsFlagged() {
            // The single most valuable thing this step produces. If deploying decreases the encoder but
            // the constants increase, every goal on the arm is the wrong way round — so the first
            // powered command drives it away from its target and into the opposite stop, at whatever
            // output the profile asked for.
            ArmTravel measured = new ArmTravel(0.0, -9.0);
            String report = measured.describe(0.0, 10.0);

            assertTrue(report.contains("SIGN MISMATCH"), report);
            assertTrue(report.contains("before powering it"), report);
        }

        @Test
        @DisplayName("Travel that disagrees with constants by more than a fifth is called out")
        void travelDisagreementIsCalledOut() {
            ArmTravel measured = new ArmTravel(0.0, 6.0);

            assertTrue(measured.describe(0.0, 10.0).contains("disagrees with constants"));
        }

        @Test
        @DisplayName("Travel that matches constants says so, and points at the powered test")
        void agreementIsStatedAndChained() {
            ArmTravel measured = new ArmTravel(0.0, 9.5);
            String report = measured.describe(0.0, 10.0);

            assertTrue(report.contains("Agrees with constants"), report);
            // The reason this step comes first: the powered travel test now has something to be checked
            // against, instead of being the only source of a number nobody can verify.
            assertTrue(report.contains("ground truth for the powered travel test"), report);
        }
    }
}
