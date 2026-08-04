package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.common.components.diagnostics.SysIdRegression.Accumulator;
import frc.robot.common.components.diagnostics.SysIdRegression.Gains;
import java.util.Random;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Tests for the on-robot SysId fit.
 *
 * <p>Every case builds samples from <b>known</b> kS, kV and kA and checks they come back out. That is
 * the only way to test a regression: a fit that returns plausible-looking numbers from real data
 * cannot be distinguished from a wrong one without ground truth.
 *
 * <p>Noise is a seeded {@link Random#nextGaussian()}. A hand-rolled generator earlier in this project
 * produced samples that were neither independent nor normal, which made a correct statistical
 * correction look broken. This fit divides by sums of squares, so a generator whose spread is not
 * what it claims would invalidate every tolerance here.
 */
class SysIdRegressionTest {

    private static final long SEED = 20260804L;

    /** Truth the synthetic data is generated from. */
    private static final double TRUE_KS = 0.18;
    private static final double TRUE_KV = 2.09;
    private static final double TRUE_KA = 0.34;

    /**
     * Feeds a realistic pair of tests: a slow ramp where acceleration is nearly zero, and a step
     * where it is not.
     *
     * @param noiseVolts Gaussian noise on the voltage samples.
     * @return the accumulator, ready to fit.
     */
    private static Accumulator syntheticRun(double noiseVolts) {
        Random random = new Random(SEED);
        Accumulator accumulator = new Accumulator();

        // Quasistatic: velocity climbing steadily, acceleration small but not exactly zero.
        for (int i = 0; i < 300; i++) {
            double velocity = 0.10 + i * 0.010;
            double acceleration = 0.010 / 0.02;
            accumulator.add(volts(velocity, acceleration, random, noiseVolts), velocity,
                    acceleration);
        }

        // Dynamic: a step, so acceleration starts high and decays as velocity approaches steady
        // state. This is the only part of the data that carries information about kA.
        double velocity = 0.10;
        for (int i = 0; i < 120; i++) {
            double acceleration = (4.0 - TRUE_KV * velocity - TRUE_KS) / TRUE_KA;
            accumulator.add(volts(velocity, acceleration, random, noiseVolts), velocity,
                    acceleration);
            velocity += acceleration * 0.02;
        }

        // Reverse, so kS is not confounded with a directional asymmetry.
        for (int i = 0; i < 300; i++) {
            double reverseVelocity = -(0.10 + i * 0.010);
            double acceleration = -0.010 / 0.02;
            accumulator.add(volts(reverseVelocity, acceleration, random, noiseVolts),
                    reverseVelocity, acceleration);
        }

        return accumulator;
    }

    /** The physical model the fit is supposed to invert. */
    private static double volts(double velocity, double acceleration, Random random,
            double noiseVolts) {
        double ideal = TRUE_KS * Math.signum(velocity) + TRUE_KV * velocity + TRUE_KA * acceleration;
        return ideal + (noiseVolts > 0 ? random.nextGaussian() * noiseVolts : 0);
    }

    @Nested
    @DisplayName("noise-free data")
    class Exact {

        @Test
        @DisplayName("recovers all three gains essentially exactly")
        void recoversGains() {
            Gains gains = syntheticRun(0).fit();

            assertEquals(TRUE_KS, gains.kS(), 1e-6);
            assertEquals(TRUE_KV, gains.kV(), 1e-6);
            assertEquals(TRUE_KA, gains.kA(), 1e-6);
            assertEquals(1.0, gains.rSquared(), 1e-6);
            assertTrue(gains.isTrustworthy());
        }
    }

    @Nested
    @DisplayName("realistically noisy data")
    class Noisy {

        @Test
        @DisplayName("recovers all three gains within a few percent")
        void recoversGains() {
            // 0.1 V of noise is a fair reflection of a SPARK voltage reading plus the finite
            // difference used for acceleration.
            Gains gains = syntheticRun(0.1).fit();

            assertEquals(TRUE_KS, gains.kS(), 0.05);
            assertEquals(TRUE_KV, gains.kV(), 0.05);
            assertEquals(TRUE_KA, gains.kA(), 0.05);
            assertTrue(gains.rSquared() > 0.99, "R2 was " + gains.rSquared());
            assertTrue(gains.isTrustworthy(), gains.describe());
        }
    }

    @Nested
    @DisplayName("a run where the dynamic test never happened")
    class QuasistaticOnly {

        /** Only the slow ramp, so acceleration is a single constant value throughout. */
        private Accumulator rampOnly() {
            Random random = new Random(SEED);
            Accumulator accumulator = new Accumulator();

            for (int i = 0; i < 300; i++) {
                double velocity = 0.10 + i * 0.010;
                double acceleration = 0.5;
                accumulator.add(volts(velocity, acceleration, random, 0), velocity, acceleration);
            }
            return accumulator;
        }

        @Test
        @DisplayName("is reported as singular rather than given a confident kA")
        void reportsSingular() {
            // With acceleration constant, the acceleration column is a multiple of the sgn(v)
            // column, so kS and kA are not separable — infinitely many pairs fit the data equally
            // well. Returning any one of them would be a confident lie.
            Gains gains = rampOnly().fit();

            assertFalse(gains.wellConditioned(), "constant acceleration must be detected");
            assertFalse(gains.isTrustworthy());
            assertTrue(gains.describe().contains("SINGULAR"), gains.describe());
        }

        @Test
        @DisplayName("says the dynamic test is what is missing")
        void explainsWhy() {
            assertTrue(rampOnly().fit().describe().contains("dynamic test"),
                    "the report has to name the fix, not just the symptom");
        }
    }

    @Nested
    @DisplayName("physically impossible fits")
    class Rejected {

        @Test
        @DisplayName("a negative kV is rejected as a wiring fault, not a bad fit")
        void negativeKvRejected() {
            Accumulator accumulator = new Accumulator();

            // Voltage and velocity disagree in sign, which is an inverted motor or encoder.
            for (int i = 0; i < 300; i++) {
                double velocity = 0.10 + i * 0.010;
                double acceleration = 0.010 / 0.02 + (i % 7) * 0.4;
                accumulator.add(-(TRUE_KV * velocity) + TRUE_KA * acceleration, velocity,
                        acceleration);
            }

            Gains gains = accumulator.fit();

            assertTrue(gains.kV() < 0);
            assertFalse(gains.isTrustworthy());
            assertTrue(gains.describe().contains("wiring"), gains.describe());
        }

        @Test
        @DisplayName("too few samples is rejected even with a perfect fit")
        void tooFewSamplesRejected() {
            Random random = new Random(SEED);
            Accumulator accumulator = new Accumulator();

            for (int i = 0; i < 10; i++) {
                double velocity = 0.5 + i * 0.3;
                double acceleration = 1.0 + i * 0.5;
                accumulator.add(volts(velocity, acceleration, random, 0), velocity, acceleration);
            }

            Gains gains = accumulator.fit();
            assertFalse(gains.isTrustworthy(), "10 samples must not be trusted: " + gains.describe());
        }
    }

    @Nested
    @DisplayName("sample filtering")
    class Filtering {

        @Test
        @DisplayName("discards samples below the velocity threshold")
        void discardsSlowSamples() {
            Accumulator accumulator = new Accumulator(0.05);

            assertFalse(accumulator.add(0.2, 0.0, 1.0), "a stationary sample carries no sign");
            assertFalse(accumulator.add(0.2, 0.01, 1.0), "below threshold");
            assertTrue(accumulator.add(0.2, 0.5, 1.0), "above threshold");

            assertEquals(1, accumulator.getSamples());
        }

        @Test
        @DisplayName("an empty accumulator does not throw")
        void emptyIsSafe() {
            Gains gains = new Accumulator().fit();

            assertEquals(0, gains.samples());
            assertFalse(gains.isTrustworthy());
            assertFalse(gains.describe().isEmpty());
        }
    }

    @Nested
    @DisplayName("the 3x3 solver")
    class Solver {

        @Test
        @DisplayName("solves a system needing a pivot swap")
        void pivotsWhenNeeded() {
            // Leading coefficient zero, so a solver without pivoting divides by zero here.
            // Constructed so the answer is exactly (1, 2, 3):
            //   0(1) + 2(2) + 1(3) = 7
            //   1(1) + 0(2) + 3(3) = 10
            //   2(1) + 1(2) + 0(3) = 4
            double[][] a = {{0, 2, 1}, {1, 0, 3}, {2, 1, 0}};
            double[] b = {7, 10, 4};

            double[] x = SysIdRegression.solve3x3(a, b);

            assertEquals(1.0, x[0], 1e-9);
            assertEquals(2.0, x[1], 1e-9);
            assertEquals(3.0, x[2], 1e-9);
        }

        @Test
        @DisplayName("returns null for a singular system rather than infinities")
        void singularReturnsNull() {
            // Third row is the sum of the first two, so the system has no unique solution.
            double[][] a = {{1, 2, 3}, {2, 4, 6}, {3, 6, 9}};
            double[] b = {1, 2, 3};

            assertNull(SysIdRegression.solve3x3(a, b));
        }
    }
}
