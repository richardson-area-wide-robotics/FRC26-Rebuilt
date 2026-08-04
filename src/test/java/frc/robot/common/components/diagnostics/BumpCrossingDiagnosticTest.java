package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.common.components.diagnostics.BumpCrossingDiagnostic.Result;
import frc.robot.common.components.diagnostics.BumpCrossingDiagnostic.Verdict;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Tests for the ramp-crossing classifier.
 *
 * <p>The three causes need <b>opposite</b> fixes — raise the current limit, lower it, or leave it
 * alone and fix the battery — so a misclassification does not merely fail to help, it sends someone
 * in the wrong direction. These pin the boundaries between them.
 */
class BumpCrossingDiagnosticTest {

    /** The configured per-motor limit in the code. */
    private static final double LIMIT = 50;

    /** A crossing where the wheels gripped and the motors sat on the limit. */
    private static Result currentLimited() {
        return BumpCrossingDiagnostic.analyse(48.5, 11.4, 0.62, 0.58, 0.04, 40, 300, LIMIT, true);
    }

    @Nested
    @DisplayName("a crossing where torque was capped by configuration")
    class CurrentLimited {

        @Test
        @DisplayName("is classified as current-limited")
        void classified() {
            assertEquals(Verdict.CURRENT_LIMITED, currentLimited().verdict());
        }

        @Test
        @DisplayName("points at the traction calibration, which is what says whether headroom exists")
        void adviceIsActionable() {
            String description = currentLimited().describe();
            assertTrue(description.contains("CURRENT-LIMITED"), description);
            assertTrue(description.contains("traction calibration"), description);
        }
    }

    @Nested
    @DisplayName("a crossing where the wheels were spinning")
    class TractionLimited {

        private Result result() {
            // Wheels at 1.4 m/s, robot at 0.3 m/s — 1.1 m/s of excess.
            return BumpCrossingDiagnostic.analyse(47.0, 11.2, 1.40, 0.30, 1.10, 35, 300, LIMIT,
                    true);
        }

        @Test
        @DisplayName("is classified as traction-limited even though the current was also pinned")
        void tractionBeatsCurrent() {
            // Both conditions hold. Traction has to win, because raising the limit here makes it
            // worse — the current reading is what it took to spin the wheels, not to climb.
            assertEquals(Verdict.TRACTION_LIMITED, result().verdict(),
                    "slip must outrank pinned current, or the advice is backwards");
        }

        @Test
        @DisplayName("says more current will not help")
        void adviceWarnsAgainstRaisingTheLimit() {
            String description = result().describe();
            assertTrue(description.contains("TRACTION-LIMITED"), description);
            assertTrue(description.contains("worse"), description);
        }
    }

    @Nested
    @DisplayName("a crossing where the pack sagged")
    class VoltageLimited {

        private Result result() {
            // Everything else looks like a textbook current-limited run, but the bus fell to 8.9 V.
            return BumpCrossingDiagnostic.analyse(49.0, 8.9, 0.55, 0.50, 0.05, 45, 300, LIMIT, true);
        }

        @Test
        @DisplayName("outranks every other verdict")
        void voltageBeatsEverything() {
            // Checked first on purpose: a sagging pack makes the other measurements untrustworthy,
            // so diagnosing it as current-limited would send someone tuning a limit against a
            // moving target.
            assertEquals(Verdict.VOLTAGE_LIMITED, result().verdict());
        }

        @Test
        @DisplayName("says to fix the battery before touching any current limit")
        void adviceOrdersTheWork() {
            String description = result().describe();
            assertTrue(description.contains("VOLTAGE-LIMITED"), description);
            assertTrue(description.contains("before changing any current limit"), description);
        }
    }

    @Nested
    @DisplayName("a crossing with no AprilTag in view")
    class SlipNotMeasurable {

        @Test
        @DisplayName("does not clear traction when it could not be measured")
        void reportsInconclusiveRatherThanFine() {
            // Nothing at a limit, but chassis speed came from the wheels, so slip reads as ~0
            // whether or not it happened. Reporting NOT_LIMITED here would be a false all-clear.
            Result result = BumpCrossingDiagnostic.analyse(22.0, 11.9, 0.90, 0.88, 0.02, 0, 300,
                    LIMIT, false);

            assertEquals(Verdict.TRACTION_NOT_MEASURABLE, result.verdict());
            assertFalse(result.slipWasMeasurable());
            assertTrue(result.describe().contains("INCONCLUSIVE"), result.describe());
        }

        @Test
        @DisplayName("still reports current-limited, which does not depend on slip")
        void currentVerdictSurvivesWithoutVision() {
            // Pinned current is a fact about configuration, measurable with no tags at all. Only
            // what to do about it depends on whether the wheels were slipping.
            Result result = BumpCrossingDiagnostic.analyse(48.0, 11.5, 0.60, 0.55, 0.02, 40, 300,
                    LIMIT, false);

            assertEquals(Verdict.CURRENT_LIMITED, result.verdict());
        }

        @Test
        @DisplayName("cannot report traction-limited, since the evidence is unavailable")
        void neverClaimsTractionWithoutEvidence() {
            // A large apparent excess with no vision is not evidence — it is noise in a
            // measurement that cannot detect what it is being asked about.
            Result result = BumpCrossingDiagnostic.analyse(20.0, 11.9, 1.50, 0.20, 1.30, 0, 300,
                    LIMIT, false);

            assertFalse(result.verdict() == Verdict.TRACTION_LIMITED,
                    "traction must never be claimed from a circular measurement");
        }
    }

    @Nested
    @DisplayName("degenerate runs")
    class Degenerate {

        @Test
        @DisplayName("a run where the robot never moved reports no crossing")
        void noCrossing() {
            Result result = BumpCrossingDiagnostic.analyse(3.0, 12.4, 0.02, 0.01, 0.01, 0, 300,
                    LIMIT, true);

            assertEquals(Verdict.NO_CROSSING_SEEN, result.verdict());
            assertTrue(result.describe().contains("NO CROSSING SEEN"), result.describe());
        }

        @Test
        @DisplayName("too few samples reports no crossing rather than a verdict")
        void tooFewSamples() {
            Result result = BumpCrossingDiagnostic.analyse(48.0, 11.0, 0.60, 0.55, 0.02, 5, 10,
                    LIMIT, true);

            assertEquals(Verdict.NO_CROSSING_SEEN, result.verdict());
        }

        @Test
        @DisplayName("a healthy crossing with headroom everywhere is reported as not limited")
        void notLimited() {
            Result result = BumpCrossingDiagnostic.analyse(24.0, 11.8, 1.20, 1.18, 0.02, 0, 300,
                    LIMIT, true);

            assertEquals(Verdict.NOT_LIMITED, result.verdict());
            assertTrue(result.describe().contains("look elsewhere"), result.describe());
        }
    }

    @Test
    @DisplayName("Total current is reported alongside per-motor, since the breaker sees the total")
    void totalCurrentIsFourTimesPerMotor() {
        Result result = currentLimited();
        assertEquals(result.peakAmpsPerMotor() * 4, result.peakTotalAmps(), 1e-9);

        // 48.5 A per motor is 194 A across four, against a 120 A main breaker. Worth having in the
        // report even when current is not the verdict.
        assertTrue(result.peakTotalAmps() > 120,
            "this scenario should show the total exceeding the breaker rating");
    }
}
