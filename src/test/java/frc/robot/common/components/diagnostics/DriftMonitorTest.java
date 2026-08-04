package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.common.components.diagnostics.DriftMonitor.Watch;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Tests for drift detection.
 *
 * <p>The behaviour that matters is restraint: an estimate built from a handful of samples must not
 * raise an alarm, and a small difference must not be reported as drift. A monitor that cries wolf
 * gets ignored, and then the real drift goes unnoticed too.
 */
class DriftMonitorTest {

  private DriftMonitor monitor;

  /** Mutable holders so a test can move the estimate and the sample count independently. */
  private double[] inUse;
  private double[] estimate;
  private int[] samples;

  @BeforeEach
  void setUp() {
    monitor = new DriftMonitor();
    inUse = new double[] {0.0762};
    estimate = new double[] {0.0762};
    samples = new int[] {0};
  }

  /** Registers a wheel-diameter watch with a 1% threshold and 100-sample minimum. */
  private Watch watchWheelDiameter() {
    return monitor.watch("drive.wheelDiameter", "m",
        () -> inUse[0], () -> estimate[0], () -> samples[0], 0.01, 100);
  }

  @Nested
  @DisplayName("Evidence gating")
  class EvidenceGating {

    @Test
    @DisplayName("No samples means no opinion")
    void noSamplesNoOpinion() {
      Watch watch = watchWheelDiameter();
      estimate[0] = 0.0700; // Would be 8% drift, well past the threshold.
      samples[0] = 0;

      assertFalse(watch.hasEnoughEvidence());
      assertFalse(watch.hasDrifted(),
          "A large apparent drift from zero samples is noise, not information");
    }

    @Test
    @DisplayName("Below the sample minimum, drift is not reported")
    void belowMinimumNotReported() {
      Watch watch = watchWheelDiameter();
      estimate[0] = 0.0700;
      samples[0] = 99;

      assertFalse(watch.hasEnoughEvidence());
      assertFalse(watch.hasDrifted());
    }

    @Test
    @DisplayName("At the sample minimum, drift is reported")
    void atMinimumReported() {
      Watch watch = watchWheelDiameter();
      estimate[0] = 0.0700;
      samples[0] = 100;

      assertTrue(watch.hasEnoughEvidence());
      assertTrue(watch.hasDrifted());
    }
  }

  @Nested
  @DisplayName("Threshold behaviour")
  class Thresholds {

    @Test
    @DisplayName("An identical estimate is not drift")
    void identicalIsNotDrift() {
      Watch watch = watchWheelDiameter();
      samples[0] = 500;

      assertEquals(0.0, watch.getDriftFraction(), 1e-12);
      assertFalse(watch.hasDrifted());
    }

    @Test
    @DisplayName("Half a percent is within tolerance for a one percent threshold")
    void smallDifferenceIsStable() {
      Watch watch = watchWheelDiameter();
      samples[0] = 500;
      estimate[0] = 0.0762 * 1.005;

      assertEquals(0.005, watch.getDriftFraction(), 1e-6);
      assertFalse(watch.hasDrifted(),
          "Reporting every half-percent wobble would train everyone to ignore this");
    }

    @Test
    @DisplayName("Two percent of tread wear is reported")
    void treadWearIsReported() {
      // The realistic case: the wheel has worn 2% smaller over a season.
      Watch watch = watchWheelDiameter();
      samples[0] = 500;
      estimate[0] = 0.0762 * 0.98;

      assertEquals(-0.02, watch.getDriftFraction(), 1e-6);
      assertTrue(watch.hasDrifted());
      assertEquals(1, monitor.getDrifted().size());
    }

    @Test
    @DisplayName("Drift is signed, so the direction is visible")
    void driftIsSigned() {
      Watch watch = watchWheelDiameter();
      samples[0] = 500;

      estimate[0] = 0.0762 * 0.97;
      assertTrue(watch.getDriftFraction() < 0, "A smaller estimate should read negative");

      estimate[0] = 0.0762 * 1.03;
      assertTrue(watch.getDriftFraction() > 0, "A larger estimate should read positive");
    }

    @Test
    @DisplayName("A zero value in use does not divide by zero")
    void zeroInUseIsSafe() {
      Watch watch = monitor.watch("odd", "", () -> 0.0, () -> 5.0, () -> 500, 0.01, 100);
      assertEquals(0.0, watch.getDriftFraction(), 1e-12);
      assertFalse(watch.hasDrifted());
    }
  }

  @Nested
  @DisplayName("Reporting")
  class Reporting {

    @Test
    @DisplayName("Several watches are tracked independently")
    void independentWatches() {
      double[] stable = {1.0};
      double[] drifted = {1.0};

      monitor.watch("stable", "", () -> stable[0], () -> 1.0, () -> 500, 0.01, 100);
      monitor.watch("drifted", "", () -> drifted[0], () -> 1.2, () -> 500, 0.01, 100);

      assertEquals(2, monitor.getWatches().size());
      assertEquals(1, monitor.getDrifted().size());
      assertEquals("drifted", monitor.getDrifted().get(0).getName());
    }

    @Test
    @DisplayName("Updating and reporting never throws, even with nothing registered")
    void emptyMonitorIsSafe() {
      org.junit.jupiter.api.Assertions.assertDoesNotThrow(() -> {
        monitor.update();
        monitor.printReport();
      });
    }

    @Test
    @DisplayName("A watch reports whether its value may be adopted automatically")
    void watchesCarryAdoptionPolicy() {
      Watch wheel = watchWheelDiameter();
      Watch noise = monitor.watch("vision.noise.xyStdDev", "m",
          () -> 0.35, () -> 0.03, () -> 900, 0.10, 100);

      assertFalse(wheel.mayAutoAdopt(),
          "Wheel diameter is measured against the pose it determines, so a human decides");
      assertTrue(noise.mayAutoAdopt(),
          "Vision noise is measured independently of itself, so adopting it closes no loop");
    }

    @Test
    @DisplayName("Report distinguishes stable, insufficient evidence, and drifted")
    void reportCoversAllThreeStates() {
      monitor.watch("stable", "", () -> 1.0, () -> 1.0, () -> 500, 0.01, 100);
      monitor.watch("unproven", "", () -> 1.0, () -> 2.0, () -> 5, 0.01, 100);
      monitor.watch("drifted", "", () -> 1.0, () -> 1.5, () -> 500, 0.01, 100);

      org.junit.jupiter.api.Assertions.assertDoesNotThrow(monitor::printReport);
      assertEquals(1, monitor.getDrifted().size(),
          "Only the one with both enough evidence and enough difference counts");
    }
  }
}
