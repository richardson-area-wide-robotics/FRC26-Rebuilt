package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.common.components.diagnostics.LoadCalibrator.Recommendation;
import java.util.Random;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Tests for the current-threshold calibration analysis.
 *
 * <p>Noise comes from a seeded {@link Random#nextGaussian()} rather than a hand-rolled generator. An
 * earlier test elsewhere in this project used a bit-sliced LCG and its samples were neither
 * independent nor normal, which made a statistical correction look wrong when it was right. The
 * analysis here divides by a standard deviation, so a generator whose spread is not what it claims
 * would make every assertion meaningless.
 */
class LoadCalibratorTest {

    /** Fixed seed: these tests assert on statistics, so they must not vary run to run. */
    private static final long SEED = 20260804L;

    /**
     * Feeds a phase of samples with gaussian noise.
     *
     * @param n     samples to add.
     * @param mean  mean current.
     * @param sigma current noise.
     * @param speed mechanism speed.
     * @param sink  where to put them.
     */
    private static void feed(int n, double mean, double sigma, double speed,
            Random random, PhaseSink sink) {
        for (int i = 0; i < n; i++) {
            sink.accept(mean + random.nextGaussian() * sigma, speed);
        }
    }

    /** A phase of the calibration to feed samples into. */
    private interface PhaseSink {
        void accept(double amps, double speed);
    }

    @Nested
    @DisplayName("a mechanism where a game piece clearly loads the motor")
    class CleanSeparation {

        private Recommendation recommend() {
            Random random = new Random(SEED);
            LoadCalibrator cal = new LoadCalibrator("INTAKE");

            // Idle at 8 A with 0.5 A of noise, running 5000 RPM free.
            feed(200, 8.0, 0.5, 5000, random, cal::addEmptySample);

            // Loaded phase: pieces present for 100 of 300 samples, at 26 A and 4000 RPM.
            // The other 200 samples are the gaps between pieces, still at idle.
            for (int i = 0; i < 300; i++) {
                boolean piecePresent = i % 3 == 0;
                cal.addLoadedSample(
                        (piecePresent ? 26.0 : 8.0) + random.nextGaussian() * 0.5,
                        piecePresent ? 4000 : 5000);
            }

            return cal.recommend();
        }

        @Test
        @DisplayName("is viable")
        void isViable() {
            assertTrue(recommend().isViable(), "18 A of excess over 0.5 A noise must be viable");
        }

        @Test
        @DisplayName("puts the work threshold between idle and loaded, not outside them")
        void thresholdSitsInTheGap() {
            Recommendation r = recommend();
            double gap = r.loadedAmps() - r.idleAmps();

            assertTrue(r.workExcessAmps() > 0, "threshold must be above idle");
            assertTrue(r.workExcessAmps() < gap,
                    "threshold " + r.workExcessAmps() + " A must be below the "
                            + gap + " A gap, or a loaded mechanism never crosses it");
        }

        @Test
        @DisplayName("recovers the free-running speed from the empty phase")
        void recoversFreeSpeed() {
            assertEquals(5000, recommend().expectedSpeed(), 1.0);
        }

        @Test
        @DisplayName("recovers what fraction of the loaded phase had a piece in it")
        void recoversDutyCycle() {
            // One sample in three was loaded. Segmentation is against idle + 3 sigma = 9.5 A, and
            // the two populations sit at 8 and 26, so essentially every loaded sample is caught and
            // essentially no idle one is.
            assertEquals(1.0 / 3.0, recommend().pieceDutyCycle(), 0.02);
        }

        @Test
        @DisplayName("measures the loaded current from the pieces, not from the whole phase")
        void loadedMeanExcludesTheGaps() {
            // The whole-phase mean is about 14 A. If segmentation were not happening, the loaded
            // figure would land there instead of at 26.
            assertEquals(26.0, recommend().loadedAmps(), 0.5);
        }
    }

    @Nested
    @DisplayName("a good mechanism fed at a realistic duty cycle")
    class SparseFeeding {

        /**
         * A strong mechanism with pieces present only a fifth of the time.
         *
         * <p>Idle 5 A with 0.5 A of noise; a piece adds 7 A. That is a 14-sigma separation, about as
         * good as a current signature gets. But the routine's own docs anticipate pieces being present
         * around 20% of a loaded phase, because a human is picking them up, so most of the phase reads
         * idle.
         */
        private Recommendation recommend() {
            Random random = new Random(SEED);
            LoadCalibrator cal = new LoadCalibrator("INTAKE");

            feed(200, 5.0, 0.5, 5000, random, cal::addEmptySample);

            // 20% of the loaded phase has a piece in it; the rest is the mechanism running empty.
            feed(60, 12.0, 0.5, 4300, random, cal::addLoadedSample);
            feed(240, 5.0, 0.5, 5000, random, cal::addLoadedSample);

            return cal.recommend();
        }

        @Test
        @DisplayName("is VIABLE -- a sparse feed must not read as a bad mechanism")
        void sparseFeedingIsStillViable() {
            // The false alarm this pins. Separation was measured on the whole loaded phase including
            // its gaps, so it scaled with duty cycle: this 14-sigma mechanism computed as about 2.8 and
            // was condemned as NOT VIABLE, with the report explicitly telling the team not to paste it.
            // Somebody then goes looking for a beam-break sensor that a working intake did not need.
            //
            // A false all-clear is bad. A false alarm that redirects the session and adds hardware is
            // worse, because nothing later contradicts it.
            Recommendation r = recommend();

            assertTrue(r.isViable(),
                "a 14-sigma mechanism fed 20% of the time must be viable; separation reported as "
                    + r.separationSigma());
        }

        @Test
        @DisplayName("and the reported duty cycle shows how sparse the feed was")
        void dutyCycleIsReported() {
            // Kept visible rather than corrected away: the operator should be able to see that the
            // phase was mostly gaps, because that is what decides whether re-running with a faster
            // feed would sharpen the numbers.
            double duty = recommend().pieceDutyCycle();

            assertTrue(duty > 0.1 && duty < 0.35,
                "about a fifth of the phase had a piece in it; got " + duty);
        }
    }

    @Nested
    @DisplayName("a mechanism where a game piece barely loads the motor")
    class OverlappingPopulations {

        private Recommendation recommend() {
            Random random = new Random(SEED);
            LoadCalibrator cal = new LoadCalibrator("SPINDEXER");

            // Idle 8 A with 2 A of noise. A piece adds only 1.5 A — well inside the noise.
            feed(200, 8.0, 2.0, 5000, random, cal::addEmptySample);
            feed(300, 9.5, 2.0, 4900, random, cal::addLoadedSample);

            return cal.recommend();
        }

        @Test
        @DisplayName("is reported as not viable rather than given a threshold")
        void isNotViable() {
            assertFalse(recommend().isViable(),
                    "1.5 A of signal under 2 A of noise cannot be separated");
        }

        @Test
        @DisplayName("says so in words")
        void explainsWhy() {
            String description = recommend().describe();
            assertTrue(description.contains("NOT VIABLE") || description.contains("INCOMPLETE"),
                    "operator must be told not to paste this: " + description);
        }

        @Test
        @DisplayName("is not rescued by the segmentation picking out noise peaks")
        void segmentationDoesNotFakeSeparation() {
            // Segmentation selects samples above idle + 3 sigma. With overlapping populations it
            // still finds some — they are noise peaks. Separation is deliberately computed on the
            // unselected phase mean so those cannot make an unusable mechanism look usable.
            Recommendation r = recommend();
            assertTrue(r.separationSigma() < 4.0,
                    "separation must be judged on the whole phase, got " + r.separationSigma());
        }
    }

    @Nested
    @DisplayName("the obstructed phase")
    class JamPhase {

        private LoadCalibrator withPhases(boolean includeJam) {
            Random random = new Random(SEED);
            LoadCalibrator cal = new LoadCalibrator("FEEDER");

            feed(200, 8.0, 0.5, 5000, random, cal::addEmptySample);

            // Working: 24 A at 4250 RPM, so 85% of free speed.
            feed(300, 24.0, 0.5, 4250, random, cal::addLoadedSample);

            if (includeJam) {
                // Obstructed: 45 A at 500 RPM, so 10% of free speed.
                feed(100, 45.0, 1.0, 500, random, cal::addJamSample);
            }

            return cal;
        }

        @Test
        @DisplayName("puts the jam threshold between working and obstructed speed")
        void thresholdSitsBetweenTheTwoPopulations() {
            Recommendation r = withPhases(true).recommend();

            assertTrue(r.jamSpeedFraction() < r.loadedSpeedFraction(),
                    "threshold " + r.jamSpeedFraction() + " must be below the working speed "
                            + r.loadedSpeedFraction() + ", or every piece reads as a jam");
            assertTrue(r.jamSpeedFraction() > r.observedJamSpeedFraction(),
                    "threshold " + r.jamSpeedFraction() + " must be above the obstructed speed "
                            + r.observedJamSpeedFraction() + ", or a real jam is never caught");
        }

        @Test
        @DisplayName("is reported as measured when it was run")
        void reportsMeasured() {
            assertTrue(withPhases(true).recommend().isJamThresholdMeasured());
        }

        @Test
        @DisplayName("is reported as inferred when it was skipped")
        void reportsInferredWhenSkipped() {
            Recommendation r = withPhases(false).recommend();

            assertFalse(r.isJamThresholdMeasured(),
                    "a threshold derived only from the working phase must not claim to be measured");
            assertTrue(r.describe().contains("INFERRED"),
                    "the report must say the jam threshold is a guess: " + r.describe());
        }

        @Test
        @DisplayName("still yields a usable threshold when skipped")
        void inferredThresholdIsStillBelowWorkingSpeed() {
            Recommendation r = withPhases(false).recommend();
            assertTrue(r.jamSpeedFraction() < r.loadedSpeedFraction(),
                    "even a guessed threshold must not fire during normal work");
        }
    }

    @Nested
    @DisplayName("incomplete runs")
    class Incomplete {

        @Test
        @DisplayName("too few empty samples is reported, not guessed around")
        void tooFewEmptySamples() {
            Random random = new Random(SEED);
            LoadCalibrator cal = new LoadCalibrator("INTAKE");

            feed(10, 8.0, 0.5, 5000, random, cal::addEmptySample);
            feed(300, 26.0, 0.5, 4000, random, cal::addLoadedSample);

            Recommendation r = cal.recommend();
            assertFalse(r.isViable());
            assertTrue(r.describe().contains("INCOMPLETE"), r.describe());
        }

        @Test
        @DisplayName("a loaded phase where no piece ever registered is reported")
        void noPieceEverRegistered() {
            Random random = new Random(SEED);
            LoadCalibrator cal = new LoadCalibrator("INTAKE");

            feed(200, 8.0, 0.5, 5000, random, cal::addEmptySample);
            // Operator ran the phase but never actually fed anything.
            feed(300, 8.0, 0.5, 5000, random, cal::addLoadedSample);

            Recommendation r = cal.recommend();
            assertFalse(r.isViable());
            assertTrue(r.describe().contains("INCOMPLETE"), r.describe());
        }

        @Test
        @DisplayName("a mechanism never run at all does not throw")
        void nothingRun() {
            Recommendation r = new LoadCalibrator("SHOOTER").recommend();

            assertFalse(r.isViable());
            assertEquals(0, r.emptySamples());
            assertFalse(r.describe().isEmpty());
        }
    }

    @Nested
    @DisplayName("segmentation")
    class Segmentation {

        @Test
        @DisplayName("attributes a sample to a piece only once the noise floor is learned")
        void needsTheEmptyPhaseFirst() {
            LoadCalibrator cal = new LoadCalibrator("INTAKE");

            // No empty phase yet, so there is no floor to segment against. Everything counts, and
            // the sample-count check is what catches the run rather than a threshold invented from
            // an unlearned baseline.
            assertTrue(cal.addLoadedSample(8.0, 5000),
                    "with no floor learned, a sample cannot be ruled out");
            assertEquals(1, cal.getLoadedSamples());
        }

        @Test
        @DisplayName("rejects a sample sitting at idle once the floor is known")
        void rejectsIdleSamples() {
            Random random = new Random(SEED);
            LoadCalibrator cal = new LoadCalibrator("INTAKE");
            feed(200, 8.0, 0.5, 5000, random, cal::addEmptySample);

            assertFalse(cal.addLoadedSample(8.0, 5000), "an idle sample is not a piece");
            assertTrue(cal.addLoadedSample(26.0, 4000), "a loaded sample is");

            assertEquals(2, cal.getLoadedPhaseSamples(), "both count towards the phase");
            assertEquals(1, cal.getLoadedSamples(), "only one counts as a piece");
        }
    }
}
