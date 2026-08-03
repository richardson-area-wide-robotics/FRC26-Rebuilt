package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import frc.robot.common.components.diagnostics.DriveCharacterization.Feedforward;
import frc.robot.common.components.diagnostics.DriveCharacterization.FeedforwardFit;
import frc.robot.common.components.diagnostics.DriveCharacterization.StraightRunResult;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Tests for the drivetrain characterisation maths.
 *
 * <p>Each case injects a known physical truth and asserts the analyser recovers it. Where the
 * real world adds noise, the synthetic data does too — the earlier wheel-scale bias slipped
 * through precisely because its tests were noise-free.
 */
class DriveCharacterizationTest {

  @Nested
  @DisplayName("Feedforward fit")
  class FeedforwardTests {

    @Test
    @DisplayName("Recovers known kS and kV from a clean sweep")
    void recoversKnownGains() {
      // Ground truth: V = 0.25 + 2.09 * v, the corrected Vortex kV.
      double trueKs = 0.25;
      double trueKv = 2.09;

      FeedforwardFit fit = new FeedforwardFit();
      for (double v = 0.5; v <= 3.0; v += 0.5) {
        fit.add(trueKs + trueKv * v, v);
      }

      Feedforward result = fit.fit();
      assertEquals(trueKs, result.kS(), 1e-6);
      assertEquals(trueKv, result.kV(), 1e-6);
      assertEquals(1.0, result.rSquared(), 1e-6);
      assertTrue(result.isTrustworthy());
    }

    @Test
    @DisplayName("Recovers gains from a noisy sweep, and reports reduced confidence")
    void recoversGainsWithNoise() {
      double trueKs = 0.30;
      double trueKv = 2.10;
      Random random = new Random(20260803L);

      FeedforwardFit fit = new FeedforwardFit();
      for (int i = 0; i < 40; i++) {
        double v = 0.3 + i * 0.07;
        double noise = random.nextGaussian() * 0.05;
        fit.add(trueKs + trueKv * v + noise, v);
      }

      Feedforward result = fit.fit();
      assertEquals(trueKv, result.kV(), 0.05, "kV should survive realistic voltage noise");
      assertEquals(trueKs, result.kS(), 0.10, "kS is the intercept, so noisier than the slope");
      assertTrue(result.rSquared() > 0.95);
      assertTrue(result.isTrustworthy());
    }

    @Test
    @DisplayName("Samples below the friction threshold are discarded")
    void ignoresStalledSamples() {
      FeedforwardFit fit = new FeedforwardFit();
      fit.add(0.5, 0.0);      // motor energised, wheels not moving
      fit.add(0.6, 0.0001);   // below threshold
      assertEquals(0, fit.size(),
          "Stalled samples carry no slope information and would drag the intercept");
    }

    @Test
    @DisplayName("A sweep with too few points is reported as untrustworthy rather than guessed")
    void tooFewSamples() {
      FeedforwardFit fit = new FeedforwardFit();
      fit.add(2.0, 1.0);
      Feedforward result = fit.fit();
      assertEquals(0, result.kV(), 1e-9);
      assertFalse(result.isTrustworthy());
    }

    @Test
    @DisplayName("A scattered sweep produces a poor R-squared and is flagged")
    void poorFitIsFlagged() {
      Random random = new Random(7L);
      FeedforwardFit fit = new FeedforwardFit();
      for (int i = 0; i < 20; i++) {
        // Velocity unrelated to voltage, as if the wheels were slipping throughout.
        fit.add(random.nextDouble() * 6.0, 0.5 + random.nextDouble() * 2.0);
      }
      Feedforward result = fit.fit();
      assertTrue(result.rSquared() < 0.9, "Slipping wheels must not look like a good fit");
      assertFalse(result.isTrustworthy());
    }
  }

  @Nested
  @DisplayName("Module-to-module variance")
  class ModuleVarianceTests {

    /** Builds a trustworthy fit with a given kV. */
    private Feedforward fitWithKv(double kv) {
      FeedforwardFit fit = new FeedforwardFit();
      for (double v = 0.5; v <= 3.0; v += 0.5) {
        fit.add(0.25 + kv * v, v);
      }
      return fit.fit();
    }

    @Test
    @DisplayName("Four matched motors report a small spread and count as uniform")
    void matchedMotorsAreUniform() {
      // 2% spread, which is normal manufacturing variation.
      DriveCharacterization.ModuleVariance variance =
          DriveCharacterization.summariseModuleVariance(new Feedforward[] {
              fitWithKv(2.09), fitWithKv(2.11), fitWithKv(2.10), fitWithKv(2.12)});

      assertEquals(4, variance.usableFits());
      assertTrue(variance.spreadPercent() < 3.0);
      assertTrue(variance.isUniform(),
          "A couple of percent of variation is expected and absorbed by the velocity loop");
      assertEquals(2.105, variance.meanKv(), 0.01);
    }

    @Test
    @DisplayName("A materially weaker corner is flagged and identified")
    void weakCornerIsIdentified() {
      // Rear-left 20% off the others: a wrong gear ratio, a worn wheel, or a tired motor.
      DriveCharacterization.ModuleVariance variance =
          DriveCharacterization.summariseModuleVariance(new Feedforward[] {
              fitWithKv(2.09), fitWithKv(2.10), fitWithKv(2.55), fitWithKv(2.11)});

      assertFalse(variance.isUniform(),
          "20% spread means one corner pulls the robot off a straight line");
      assertEquals(2, variance.worstModule(), "Rear-left is index 2 in FL, FR, RL, RR order");
      assertTrue(variance.spreadPercent() > 15);
    }

    @Test
    @DisplayName("Min and max bracket every usable fit")
    void minMaxBracketTheFits() {
      DriveCharacterization.ModuleVariance variance =
          DriveCharacterization.summariseModuleVariance(new Feedforward[] {
              fitWithKv(2.00), fitWithKv(2.20), fitWithKv(2.10), fitWithKv(2.15)});

      assertEquals(2.00, variance.minKv(), 0.01);
      assertEquals(2.20, variance.maxKv(), 0.01);
      assertTrue(variance.meanKv() > variance.minKv());
      assertTrue(variance.meanKv() < variance.maxKv());
    }

    @Test
    @DisplayName("Untrustworthy fits are excluded rather than dragging the mean")
    void poorFitsAreExcluded() {
      Feedforward poor = new Feedforward(0, 0, 0, 0);
      DriveCharacterization.ModuleVariance variance =
          DriveCharacterization.summariseModuleVariance(new Feedforward[] {
              fitWithKv(2.10), poor, fitWithKv(2.12), fitWithKv(2.11)});

      assertEquals(3, variance.usableFits());
      assertEquals(2.11, variance.meanKv(), 0.02,
          "A zero-kV failed fit must not pull the mean down");
    }

    @Test
    @DisplayName("Nulls are tolerated, for a sweep that never ran")
    void nullsAreTolerated() {
      DriveCharacterization.ModuleVariance variance =
          DriveCharacterization.summariseModuleVariance(new Feedforward[] {null, null, null, null});

      assertEquals(0, variance.usableFits());
      assertEquals(-1, variance.worstModule());
      assertFalse(variance.isUniform());
    }

    @Test
    @DisplayName("Too few usable fits is never called uniform")
    void tooFewFitsIsNotUniform() {
      DriveCharacterization.ModuleVariance variance =
          DriveCharacterization.summariseModuleVariance(new Feedforward[] {
              fitWithKv(2.10), null, null, null});

      assertEquals(1, variance.usableFits());
      assertFalse(variance.isUniform(),
          "One module tells you nothing about whether the four agree");
    }
  }

  @Nested
  @DisplayName("Scale factors")
  class ScaleTests {

    @Test
    @DisplayName("Wheel scale under 1.0 when odometry over-reports")
    void wheelScaleDetectsOverReporting() {
      // Encoders claim 3.00 m, tags say 2.94 m: the wheels are 2% smaller than configured.
      assertEquals(0.98, DriveCharacterization.wheelScale(2.94, 3.00), 1e-9);
    }

    @Test
    @DisplayName("Wheel scale withheld when the run was too short to be meaningful")
    void wheelScaleNeedsDistance() {
      assertEquals(1.0, DriveCharacterization.wheelScale(0.2, 0.2), 1e-9);
    }

    @Test
    @DisplayName("A 3 inch wheel reading 2% small matches the expected diameter correction")
    void wheelScaleAppliesToDiameter() {
      double nominal = 0.0762; // exactly 3.00 in
      double scale = DriveCharacterization.wheelScale(2.94, 3.00);
      double corrected = nominal * scale;
      assertEquals(0.074676, corrected, 1e-6);
      assertTrue(corrected < nominal, "Tread compression makes the effective wheel smaller");
    }

    @Test
    @DisplayName("Gyro scale recovered from a multi-turn spin")
    void gyroScaleFromSpin() {
      // Gyro reports 1080 deg over three turns, tags say 1069.2: gyro reads 1% high.
      assertEquals(0.99, DriveCharacterization.gyroScale(1069.2, 1080.0), 1e-9);
    }

    @Test
    @DisplayName("Gyro scale withheld for rotations too small to beat tag noise")
    void gyroScaleNeedsRotation() {
      assertEquals(1.0, DriveCharacterization.gyroScale(44.0, 45.0), 1e-9);
    }

    @Test
    @DisplayName("Effective drive radius from module speed and rotation rate")
    void driveRadiusFromSpin() {
      // A module at 0.4 m/s while the chassis turns at 1 rad/s sits 0.4 m from centre.
      assertEquals(0.4, DriveCharacterization.effectiveDriveRadius(0.4, 1.0), 1e-9);
      // Sign of rotation must not matter.
      assertEquals(0.4, DriveCharacterization.effectiveDriveRadius(0.4, -1.0), 1e-9);
    }

    @Test
    @DisplayName("Drive radius withheld when barely rotating")
    void driveRadiusNeedsRotation() {
      assertEquals(0, DriveCharacterization.effectiveDriveRadius(0.4, 0.05), 1e-9);
    }

    @Test
    @DisplayName("Geometric radius matches the 26.5 inch square frame")
    void geometricRadiusMatchesFrame() {
      double side = 0.6731; // 26.5 in
      double expected = Math.hypot(side / 2, side / 2);
      assertEquals(expected, DriveCharacterization.geometricDriveRadius(side, side), 1e-9);
      assertEquals(0.4759, expected, 1e-4);
    }
  }

  @Nested
  @DisplayName("Steering alignment")
  class SteeringTests {

    @Test
    @DisplayName("Misalignment is the difference between commanded and actual travel")
    void recoversOffset() {
      assertEquals(0.5,
          DriveCharacterization.commonModeSteerOffsetDegrees(0.0, 0.5), 1e-9);
      assertEquals(-0.5,
          DriveCharacterization.commonModeSteerOffsetDegrees(0.0, -0.5), 1e-9);
    }

    @Test
    @DisplayName("Misalignment wraps correctly near 180 degrees")
    void offsetWraps() {
      assertEquals(-2.0,
          DriveCharacterization.commonModeSteerOffsetDegrees(179.0, 177.0), 1e-9);
      assertEquals(2.0,
          DriveCharacterization.commonModeSteerOffsetDegrees(179.0, -179.0), 1e-9);
    }

    @Test
    @DisplayName("Half a degree of misalignment consumes an entire 1 inch budget over 10 ft")
    void halfDegreeEatsTheBudget() {
      // This is the whole reason steering alignment is measured: the Through Bore V2 is
      // specified to +/-0.5 deg, and that alone is more than the requirement allows.
      double crossTrack = DriveCharacterization.crossTrackErrorFromSteerOffset(0.5, 3.048);
      assertEquals(0.0266, crossTrack, 0.0005);
      assertTrue(crossTrack > 0.0254,
          "0.5 deg over 10 ft is " + crossTrack * 1000 + " mm, which exceeds the 25.4 mm budget");
    }

    @Test
    @DisplayName("The misalignment the budget can actually tolerate is under half a degree")
    void toleranceImpliesTightAlignment() {
      double allowed = Math.toDegrees(Math.asin(0.0254 / 3.048));
      assertEquals(0.477, allowed, 0.005,
          "Alignment must be held to better than ~0.48 deg to meet the requirement on this "
              + "term alone, and that assumes every other term is zero");
    }
  }

  @Nested
  @DisplayName("Gain proposals")
  class GainProposalTests {

    @Test
    @DisplayName("Gain scales up when the measured error exceeds the target")
    void scalesUp() {
      assertEquals(0.36, DriveCharacterization.proposeProportionalGain(0.18, 0.20, 0.10), 1e-9);
    }

    @Test
    @DisplayName("Gain scales down when the error is already better than target")
    void scalesDown() {
      assertEquals(0.09, DriveCharacterization.proposeProportionalGain(0.18, 0.05, 0.10), 1e-9);
    }

    @Test
    @DisplayName("A wild measurement cannot produce a wild gain")
    void clampsExtremes() {
      assertEquals(0.18 * 4, DriveCharacterization.proposeProportionalGain(0.18, 100.0, 0.10),
          1e-9);
      assertEquals(0.18 * 0.25,
          DriveCharacterization.proposeProportionalGain(0.18, 0.0001, 0.10), 1e-9);
    }

    @Test
    @DisplayName("Nonsense inputs leave the gain untouched")
    void invalidInputsAreInert() {
      assertEquals(0.18, DriveCharacterization.proposeProportionalGain(0.18, 0, 0.10), 1e-9);
      assertEquals(0.18, DriveCharacterization.proposeProportionalGain(0.18, 0.2, 0), 1e-9);
    }
  }

  @Nested
  @DisplayName("Straight run analysis")
  class StraightRunTests {

    @Test
    @DisplayName("A perfect run has no error in either axis")
    void perfectRun() {
      StraightRunResult result =
          DriveCharacterization.analyseStraightRun(3.048, 0.0, 3.048, 0.0, 0.0);
      assertEquals(0.0, result.alongTrackErrorMeters(), 1e-9);
      assertEquals(0.0, result.crossTrackErrorMeters(), 1e-9);
      assertTrue(result.meetsTolerance(0.0254));
    }

    @Test
    @DisplayName("Falling short shows as negative along-track error only")
    void shortRun() {
      StraightRunResult result =
          DriveCharacterization.analyseStraightRun(3.048, 0.0, 3.020, 0.0, 0.0);
      assertEquals(-0.028, result.alongTrackErrorMeters(), 1e-6);
      assertEquals(0.0, result.crossTrackErrorMeters(), 1e-9);
      assertFalse(result.meetsTolerance(0.0254), "28 mm short exceeds a 25.4 mm budget");
    }

    @Test
    @DisplayName("Drifting sideways shows as cross-track error only")
    void sidewaysDrift() {
      StraightRunResult result =
          DriveCharacterization.analyseStraightRun(3.048, 0.0, 3.048, 0.030, 0.0);
      assertEquals(0.0, result.alongTrackErrorMeters(), 1e-9);
      assertEquals(0.030, result.crossTrackErrorMeters(), 1e-9);
      assertFalse(result.meetsTolerance(0.0254));
    }

    @Test
    @DisplayName("Errors are decomposed correctly when driving at an angle")
    void decomposesAtAnAngle() {
      // Commanded 90 degrees, so along-track is field +y and cross-track is field −x.
      StraightRunResult result =
          DriveCharacterization.analyseStraightRun(3.048, 90.0, -0.020, 3.048, 0.0);
      assertEquals(0.0, result.alongTrackErrorMeters(), 1e-6);
      assertEquals(0.020, result.crossTrackErrorMeters(), 1e-6);
    }

    @Test
    @DisplayName("Total error combines both axes")
    void totalCombinesAxes() {
      StraightRunResult result =
          DriveCharacterization.analyseStraightRun(3.048, 0.0, 3.048 + 0.015, 0.020, 0.0);
      assertEquals(Math.hypot(0.015, 0.020), result.totalErrorMeters(), 1e-9);
      assertEquals(0.025, result.totalErrorMeters(), 1e-6);
      assertTrue(result.meetsTolerance(0.0254), "25 mm just fits inside a 25.4 mm budget");
    }

    @Test
    @DisplayName("A 1% wheel scale error alone fails the 10 ft requirement")
    void onePercentScaleFailsTheSpec() {
      // The point of the whole exercise: uncalibrated odometry cannot meet this budget.
      double actual = 3.048 * 0.99;
      StraightRunResult result =
          DriveCharacterization.analyseStraightRun(3.048, 0.0, actual, 0.0, 0.0);
      assertEquals(-0.0305, result.alongTrackErrorMeters(), 0.0005);
      assertFalse(result.meetsTolerance(0.0254),
          "1% scale error is 30.5 mm over 10 ft, already over the 25.4 mm budget");
    }
  }
}
