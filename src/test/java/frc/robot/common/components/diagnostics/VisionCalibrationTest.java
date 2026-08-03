package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.common.components.diagnostics.VisionCalibration.RunningStats;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Tests for the AprilTag calibration maths.
 *
 * <p>Synthetic samples with a known, deliberately injected error let us assert that the
 * analyser recovers exactly that error — which is the only way to know the numbers it
 * reports on the real field mean what they claim.
 */
class VisionCalibrationTest {

  private VisionCalibration calibration;

  @BeforeEach
  void setUp() {
    calibration = new VisionCalibration("TestCalibration");
  }

  @Nested
  @DisplayName("Running statistics")
  class Statistics {

    @Test
    @DisplayName("Mean and standard deviation match the textbook result")
    void welfordMatchesTextbook() {
      RunningStats stats = new RunningStats();
      for (double v : new double[] {2, 4, 4, 4, 5, 5, 7, 9}) {
        stats.add(v);
      }

      assertEquals(8, stats.getCount());
      assertEquals(5.0, stats.getMean(), 1e-9);
      // Sample standard deviation of that classic set is sqrt(32/7).
      assertEquals(Math.sqrt(32.0 / 7.0), stats.getStdDev(), 1e-9);
      assertEquals(2.0, stats.getMin(), 1e-9);
      assertEquals(9.0, stats.getMax(), 1e-9);
    }

    @Test
    @DisplayName("A single sample has zero standard deviation, not NaN")
    void singleSampleIsSafe() {
      RunningStats stats = new RunningStats();
      stats.add(42.0);
      assertEquals(42.0, stats.getMean(), 1e-9);
      assertEquals(0.0, stats.getStdDev(), 1e-9);
    }

    @Test
    @DisplayName("An empty set returns zeros rather than NaN")
    void emptyIsSafe() {
      RunningStats stats = new RunningStats();
      assertEquals(0, stats.getCount());
      assertEquals(0.0, stats.getMean(), 1e-9);
      assertEquals(0.0, stats.getStdDev(), 1e-9);
    }
  }

  @Nested
  @DisplayName("Wheel scale estimation")
  class WheelScale {

    @Test
    @DisplayName("Perfect odometry yields a scale of exactly 1.0")
    void perfectOdometryIsUnity() {
      driveStraight(1.0, 20);
      assertEquals(1.0, calibration.getWheelScaleEstimate(), 1e-6);
    }

    @Test
    @DisplayName("Odometry over-reporting by 2% is recovered as a 0.98 scale")
    void detectsOverReporting() {
      // Wheels bigger than configured: odometry claims more distance than really travelled.
      driveStraight(0.98, 40);

      assertTrue(calibration.isWheelScaleTrustworthy(),
          "40 samples of 0.5 m should be plenty of distance");
      assertEquals(0.98, calibration.getWheelScaleEstimate(), 0.005,
          "The recovered scale is what kWheelDiameterMeters should be multiplied by");
    }

    @Test
    @DisplayName("Odometry under-reporting by 3% is recovered as a 1.03 scale")
    void detectsUnderReporting() {
      driveStraight(1.03, 40);
      assertEquals(1.03, calibration.getWheelScaleEstimate(), 0.005);
    }

    @Test
    @DisplayName("The estimate stays at 1.0 until enough distance has been driven")
    void withholdsEstimateUntilConfident() {
      driveStraight(0.9, 1);
      assertEquals(1.0, calibration.getWheelScaleEstimate(), 1e-9,
          "A single short segment must not produce a confident-looking correction");
      assertFalse(calibration.isWheelScaleTrustworthy());
    }

    @Test
    @DisplayName("Movement totalling less than the minimum segment contributes nothing")
    void ignoresMovementBelowNoiseFloor() {
      // Four 5 cm steps is 20 cm total — under the 30 cm floor, where vision noise would
      // swamp the signal.
      for (int i = 0; i < 5; i++) {
        double d = i * 0.05;
        calibration.addSample(pose(d, 0), pose(d, 0), pose(d, 0), 0.0, 2.0, 0.02, 1.0);
      }
      assertEquals(0.0, calibration.getCalibrationDistanceMeters(), 1e-9,
          "Movement below the minimum segment must contribute no distance");
    }

    @Test
    @DisplayName("Small steps are batched, so slow driving still calibrates")
    void batchesSmallStepsRatherThanDiscardingThem() {
      // 5 cm steps over 5 m total. Each individual step is below the floor, but the
      // reference sample is only advanced when a segment is accepted, so the movement
      // accumulates in ~30 cm chunks instead of being thrown away. A robot that only ever
      // creeps should still end up calibrated.
      for (int i = 0; i <= 100; i++) {
        double d = i * 0.05;
        calibration.addSample(pose(d, 0), pose(d, 0), pose(d, 0), 0.0, 2.0, 0.02, 1.0);
      }

      double accumulated = calibration.getCalibrationDistanceMeters();
      assertTrue(accumulated > 4.0,
          "Creeping 5 m in 5 cm steps should still accumulate most of that distance, got "
              + accumulated);
      assertTrue(accumulated <= 5.0 + 1e-9, "Cannot accumulate more than was travelled");
      assertEquals(1.0, calibration.getWheelScaleEstimate(), 1e-6,
          "Batching must not bias the ratio when odometry and vision agree");
    }

    @Test
    @DisplayName("Standing still contributes nothing to the wheel scale")
    void stationaryDoesNotAffectScale() {
      for (int i = 0; i < 30; i++) {
        calibration.addSample(pose(5, 5), pose(5, 5), pose(5, 5), 0.0, 2.0, 0.02, 0.0);
      }
      assertEquals(0.0, calibration.getCalibrationDistanceMeters(), 1e-9);
      assertEquals(1.0, calibration.getWheelScaleEstimate(), 1e-9);
    }

    /**
     * Feeds straight-line motion where vision travels {@code visionRatio} times as far as
     * odometry claims.
     */
    private void driveStraight(double visionRatio, int samples) {
      for (int i = 0; i <= samples; i++) {
        double odometryDistance = i * 0.5;
        double visionDistance = odometryDistance * visionRatio;
        calibration.addSample(
            pose(visionDistance, 0),
            pose(odometryDistance, 0),
            pose(visionDistance, 0),
            0.0,
            2.5,
            0.03,
            1.5);
      }
    }
  }

  @Nested
  @DisplayName("Gyro error")
  class GyroError {

    @Test
    @DisplayName("A constant gyro offset is reported as the mean error")
    void detectsConstantOffset() {
      for (int i = 0; i < 20; i++) {
        // Tags say 30 degrees, gyro says 25: a 5 degree error.
        calibration.addSample(
            poseWithYaw(1, 1, 30.0), pose(1, 1), pose(1, 1), 25.0, 2.0, 0.02, 0.0);
      }
      assertEquals(5.0, calibration.getGyroYawError().getMean(), 1e-6);
    }

    @Test
    @DisplayName("Yaw error wraps correctly across the 180 degree boundary")
    void wrapsAcrossDiscontinuity() {
      // Tags at 179, gyro at -179: the true error is −2, not +358.
      calibration.addSample(
          poseWithYaw(1, 1, 179.0), pose(1, 1), pose(1, 1), -179.0, 2.0, 0.02, 0.0);
      assertEquals(-2.0, calibration.getGyroYawError().getMean(), 1e-6);
    }

    @Test
    @DisplayName("Growing gyro drift shows up as a spread, not just an offset")
    void detectsDrift() {
      for (int i = 0; i < 20; i++) {
        double drift = i * 0.5;
        calibration.addSample(
            poseWithYaw(1, 1, drift), pose(1, 1), pose(1, 1), 0.0, 2.0, 0.02, 0.0);
      }
      assertTrue(calibration.getGyroYawError().getStdDev() > 1.0,
          "Accumulating drift must widen the distribution, distinguishing it from a fixed "
              + "mounting offset");
      assertTrue(calibration.getGyroYawError().getMax() > 9.0);
    }
  }

  @Nested
  @DisplayName("Vision noise measurement")
  class NoiseMeasurement {

    @Test
    @DisplayName("Stationary scatter is recovered as the measured standard deviation")
    void measuresStationaryNoise() {
      // Alternate +/-5 cm about a fixed point: population spread is 5 cm.
      for (int i = 0; i < 40; i++) {
        double jitter = (i % 2 == 0) ? 0.05 : -0.05;
        calibration.addSample(
            pose(4.0 + jitter, 3.0 + jitter),
            pose(4.0, 3.0),
            pose(4.0, 3.0),
            0.0, 2.0, 0.02,
            0.0);
      }

      double measured = calibration.getMeasuredXyStdDevMeters();
      assertEquals(0.05, measured, 0.005,
          "This is the number to put into the pose estimator instead of a guess");
    }

    @Test
    @DisplayName("Noise is not reported until there are enough samples to mean anything")
    void withholdsNoiseUntilEnoughSamples() {
      for (int i = 0; i < 5; i++) {
        calibration.addSample(pose(1, 1), pose(1, 1), pose(1, 1), 0.0, 2.0, 0.02, 0.0);
      }
      assertEquals(0.0, calibration.getMeasuredXyStdDevMeters(), 1e-9);
      assertEquals(0.0, calibration.getMeasuredYawStdDevDegrees(), 1e-9);
    }

    @Test
    @DisplayName("Rotational noise is measured separately from translational")
    void measuresYawNoise() {
      for (int i = 0; i < 40; i++) {
        double jitter = (i % 2 == 0) ? 2.0 : -2.0;
        calibration.addSample(
            poseWithYaw(4, 3, jitter), pose(4, 3), pose(4, 3), 0.0, 2.0, 0.02, 0.0);
      }
      assertEquals(2.0, calibration.getMeasuredYawStdDevDegrees(), 0.2);
    }
  }

  @Nested
  @DisplayName("Bookkeeping")
  class Bookkeeping {

    @Test
    @DisplayName("Latency and tag distance are tracked")
    void tracksLatencyAndDistance() {
      calibration.addSample(pose(1, 1), pose(1, 1), pose(1, 1), 0.0, 3.5, 0.040, 0.0);
      calibration.addSample(pose(1, 1), pose(1, 1), pose(1, 1), 0.0, 1.5, 0.020, 0.0);

      assertEquals(0.030, calibration.getLatency().getMean(), 1e-9);
      assertEquals(0.040, calibration.getLatency().getMax(), 1e-9);
      assertEquals(2, calibration.getSampleCount());
    }

    @Test
    @DisplayName("Fused residual measures how far the estimate sits from ground truth")
    void tracksFusedResidual() {
      // Fused pose is 20 cm from the tag-derived truth.
      calibration.addSample(pose(1.0, 0), pose(1.0, 0), pose(1.2, 0), 0.0, 2.0, 0.02, 0.0);
      assertEquals(0.2, calibration.getFusedResidual().getMean(), 1e-9);
    }

    @Test
    @DisplayName("Null samples are rejected rather than throwing")
    void nullSamplesRejected() {
      calibration.addSample(null, pose(1, 1), pose(1, 1), 0.0, 2.0, 0.02, 0.0);
      assertEquals(0, calibration.getSampleCount());
    }

    @Test
    @DisplayName("Reset clears every accumulated figure")
    void resetClearsEverything() {
      for (int i = 0; i < 30; i++) {
        calibration.addSample(
            pose(i * 0.5, 0), pose(i * 0.5, 0), pose(i * 0.5, 0), 5.0, 2.0, 0.02, 1.5);
      }
      assertTrue(calibration.getSampleCount() > 0);

      calibration.reset();

      assertEquals(0, calibration.getSampleCount());
      assertEquals(1.0, calibration.getWheelScaleEstimate(), 1e-9);
      assertEquals(0.0, calibration.getCalibrationDistanceMeters(), 1e-9);
      assertEquals(0, calibration.getGyroYawError().getCount());
    }
  }

  private static Pose2d pose(double x, double y) {
    return new Pose2d(x, y, new Rotation2d());
  }

  private static Pose2d poseWithYaw(double x, double y, double yawDegrees) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(yawDegrees));
  }
}
