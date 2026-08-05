package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.common.components.diagnostics.RotationalInertiaCalibrator.IntakeState;
import frc.robot.common.components.diagnostics.RotationalInertiaCalibrator.Result;
import frc.robot.common.components.diagnostics.RotationalInertiaCalibrator.SlopeFit;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import java.util.Random;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Tests for measuring rotational inertia from a spin.
 *
 * <p>Built the only way a measurement can honestly be tested: pick a known inertia, generate the
 * angular rate a real robot with that inertia would produce, and check the number comes back out.
 *
 * <p>This matters more than usual because CAD cannot supply this value on this robot, so there is no
 * independent figure to sanity-check the result against. The arithmetic has to be right on its own.
 */
class RotationalInertiaCalibratorTest {

    private static final long SEED = 20260804L;

    /** A plausible robot inertia, in the range the lumped-mass estimate suggests. */
    private static final double TRUE_MOI = 3.90;

    /** Current per motor during the synthetic spin. */
    private static final double AMPS = 15.0;

    @Nested
    @DisplayName("the torque model")
    class Torque {

        @Test
        @DisplayName("scales linearly with current")
        void linearInCurrent() {
            double at10 = RotationalInertiaCalibrator.torqueFromCurrent(10);
            double at20 = RotationalInertiaCalibrator.torqueFromCurrent(20);

            assertEquals(2.0, at20 / at10, 1e-9);
        }

        @Test
        @DisplayName("matches a hand calculation for the as-built drivetrain")
        void matchesHandCalculation() {
            // 4 motors x (0.01706 N.m/A x 15 A x 4.50 reduction / 0.0381 m wheel radius) x 0.42207 m
            // drive radius. Worked through: 1.1948 N.m at the wheel shaft per motor, 30.20 N at the
            // carpet, 120.8 N total, 51.0 N.m about the centre.
            assertEquals(51.0, RotationalInertiaCalibrator.torqueFromCurrent(AMPS), 0.5);
        }

        @Test
        @DisplayName("uses the drive radius, so it tracks the corrected module spacing")
        void tracksModuleSpacing() {
            // 23.5 in module square -> 0.42207 m. If this drifts, either the spacing changed or the
            // 26.5 in frame-perimeter value is back.
            assertEquals(0.42207, SwerveDriveSubsystem.getDriveRadiusMeters(), 1e-5);
        }
    }

    @Nested
    @DisplayName("recovering a known inertia")
    class Recovery {

        /**
         * Builds a spin whose angular rate is what a robot of {@link #TRUE_MOI} would actually do.
         *
         * @param noiseRadPerSec Gaussian noise on the rate, as a gyro would have.
         * @return the fitted result.
         */
        private Result syntheticSpin(double noiseRadPerSec) {
            Random random = new Random(SEED);
            SlopeFit fit = new SlopeFit();

            double torque = RotationalInertiaCalibrator.torqueFromCurrent(AMPS);
            double alpha = torque / TRUE_MOI;

            // 0.6 s at 50 Hz, which is what the routine collects.
            for (int i = 0; i < 30; i++) {
                double t = i * 0.02;
                double omega = alpha * t + (noiseRadPerSec > 0
                        ? random.nextGaussian() * noiseRadPerSec
                        : 0);
                fit.add(t, omega);
            }

            return new Result(IntakeState.STOWED,
                    RotationalInertiaCalibrator.estimateMoi(AMPS, fit.slope()),
                    torque, fit.slope(), AMPS, fit.rSquared(), 1.0, fit.count());
        }

        @Test
        @DisplayName("exactly, from clean data")
        void exactFromCleanData() {
            Result result = syntheticSpin(0);

            assertEquals(TRUE_MOI, result.momentOfInertia(), 1e-6);
            assertTrue(result.isValid(), result.describe());
        }

        @Test
        @DisplayName("within a few percent from noisy gyro data")
        void closeFromNoisyData() {
            // 0.05 rad/s of noise is roughly a navX2's rate noise.
            Result result = syntheticSpin(0.05);

            assertEquals(TRUE_MOI, result.momentOfInertia(), 0.2);
            assertTrue(result.rSquared() > 0.95, "R2 was " + result.rSquared());
            assertTrue(result.isValid(), result.describe());
        }

        @Test
        @DisplayName("a stationary robot yields zero rather than infinity")
        void stationaryIsSafe() {
            assertEquals(0.0, RotationalInertiaCalibrator.estimateMoi(AMPS, 0.0), 1e-9);
            assertFalse(new Result(IntakeState.STOWED, 0, 51, 0, AMPS, 0, 1.0, 30).isValid());
        }
    }

    @Nested
    @DisplayName("wheel slip")
    class Slip {

        @Test
        @DisplayName("invalidates the run, because torque no longer reaches the carpet")
        void slipInvalidates() {
            // Everything else looks fine, but the wheels ran 40% faster than the spin rate implies.
            // The inertia would come out high, and plausibly so — which is why this has to be a
            // rejection rather than a warning.
            Result result = new Result(IntakeState.STOWED, 5.5, 51, 9.3, AMPS, 0.99, 1.40, 30);

            assertFalse(result.isValid());
            assertTrue(result.describe().contains("slipping"), result.describe());
            assertTrue(result.describe().contains("too high"), result.describe());
        }

        @Test
        @DisplayName("a small excess is not slip")
        void smallExcessIsTolerated() {
            // Wheel-scale error and gyro lag both produce a few percent. Neither is slip.
            Result result = new Result(IntakeState.STOWED, TRUE_MOI, 51, 13.1, AMPS, 0.99, 1.05, 30);
            assertTrue(result.isValid(), result.describe());
        }
    }

    @Nested
    @DisplayName("the slope fit")
    class Slope {

        @Test
        @DisplayName("keeps the near-zero samples a feedforward fit would discard")
        void keepsSlowSamples() {
            // The start of the rise is where the inertia information is. FeedforwardFit drops
            // velocities under 1e-3, which is right for a sweep and would silently remove the most
            // informative samples here.
            SlopeFit fit = new SlopeFit();
            fit.add(0.00, 0.0);
            fit.add(0.02, 0.2);
            fit.add(0.04, 0.4);

            assertEquals(3, fit.count());
            assertEquals(10.0, fit.slope(), 1e-9);
        }

        @Test
        @DisplayName("a perfect line gives an R2 of 1")
        void perfectLine() {
            SlopeFit fit = new SlopeFit();
            for (int i = 0; i < 10; i++) {
                fit.add(i * 0.02, 7.5 * i * 0.02);
            }

            assertEquals(7.5, fit.slope(), 1e-9);
            assertEquals(1.0, fit.rSquared(), 1e-9);
        }

        @Test
        @DisplayName("too few samples yields zero rather than a spurious slope")
        void tooFewSamples() {
            SlopeFit fit = new SlopeFit();
            fit.add(0.0, 0.0);

            assertEquals(0.0, fit.slope(), 1e-9);
            assertEquals(0.0, fit.rSquared(), 1e-9);
        }
    }

    @Nested
    @DisplayName("the intake states")
    class IntakeStates {

        @Test
        @DisplayName("a deployed intake should measure a larger inertia than a stowed one")
        void deployedIsLarger() {
            // Not a property of the code — a property of the robot, recorded so the expected sign of
            // the difference is written down. Deploying moves mass outward and inertia goes as
            // radius squared, so a measured decrease means something is wrong with the run.
            double stowed = 3.90;
            double deployed = 4.70;

            assertTrue(deployed > stowed,
                "deploying the intake moves mass away from the centre, so I must increase");

            double percent = 100 * (deployed - stowed) / stowed;
            assertTrue(percent > 10,
                "a change this size means one robotMOI cannot serve both states; got " + percent);
        }
    }
}
