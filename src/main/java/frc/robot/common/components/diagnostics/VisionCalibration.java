package frc.robot.common.components.diagnostics;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.Logger;

/**
 * Turns AprilTag sightings into the numbers you need to calibrate the robot's constants.
 *
 * <p>AprilTags are surveyed to known field coordinates, so a tag-derived pose is effectively
 * <em>ground truth</em>. Wheel odometry is not: it drifts from wheel wear, tread compression,
 * scrub and slip. Comparing the two over a run measures errors that are otherwise invisible,
 * and each measurement maps onto a specific constant you can then correct:
 *
 * <ul>
 *   <li><b>Wheel scale</b> — if odometry says the robot travelled 10.00 m while the tags say
 *       9.80 m, the effective wheel diameter is 2% too large. Multiply
 *       {@code kWheelDiameterMeters} by {@link #getWheelScaleEstimate()}.</li>
 *   <li><b>Gyro yaw error</b> — accumulated difference between tag-measured heading and the
 *       NavX. A steady ramp is gyro scale error; a slow creep while stationary is drift.</li>
 *   <li><b>Vision noise</b> — the spread of tag estimates while the robot is stationary
 *       <em>is</em> the standard deviation the pose estimator should be told about. This
 *       replaces guessed values with measured ones.</li>
 *   <li><b>Latency</b> — how stale a measurement is by the time the roboRIO acts on it.</li>
 * </ul>
 *
 * <p>Pure computation: no hardware, no WPILib subsystem, no side effects beyond logging. That
 * makes it directly unit-testable with synthetic samples, which is how the maths here is
 * verified.
 */
public class VisionCalibration {

  /** Welford's online mean and variance — numerically stable, no sample retention. */
  public static final class RunningStats {
    private int count;
    private double mean;
    private double m2;
    private double min = Double.POSITIVE_INFINITY;
    private double max = Double.NEGATIVE_INFINITY;

    /** @param value Sample to fold in. */
    public void add(double value) {
      count++;
      double delta = value - mean;
      mean += delta / count;
      m2 += delta * (value - mean);
      min = Math.min(min, value);
      max = Math.max(max, value);
    }

    public int getCount() {
      return count;
    }

    public double getMean() {
      return count == 0 ? 0 : mean;
    }

    /** @return sample standard deviation, or 0 with fewer than two samples. */
    public double getStdDev() {
      return count < 2 ? 0 : Math.sqrt(m2 / (count - 1));
    }

    public double getMin() {
      return count == 0 ? 0 : min;
    }

    public double getMax() {
      return count == 0 ? 0 : max;
    }

    void reset() {
      count = 0;
      mean = 0;
      m2 = 0;
      min = Double.POSITIVE_INFINITY;
      max = Double.NEGATIVE_INFINITY;
    }
  }

  /**
   * Minimum odometry movement between samples before it counts toward the wheel-scale ratio.
   *
   * <p>Vision noise is a fixed number of centimetres regardless of distance travelled, so
   * short hops have a terrible signal-to-noise ratio. Only comparing longer moves keeps the
   * estimate meaningful.
   */
  private static final double MIN_SEGMENT_METERS = 0.30;

  /** Above this speed the robot is not considered stationary, in m/s. */
  private static final double STATIONARY_SPEED_THRESHOLD = 0.05;

  private final String logRoot;

  // Wheel scale accumulation.
  private double odometryPathMeters;
  private double visionPathMeters;
  private Pose2d lastOdometrySample;
  private Pose2d lastVisionSample;

  /**
   * Individual measured vision segment lengths, retained so they can be noise-corrected once
   * the noise level is known.
   *
   * <p>A calibration run produces one entry per ~30 cm travelled, so a few hundred for a
   * long run — small enough to keep.
   */
  private final List<Double> visionSegments = new ArrayList<>();

  // Statistics.
  private final RunningStats gyroYawErrorDeg = new RunningStats();
  private final RunningStats stationaryXMeters = new RunningStats();
  private final RunningStats stationaryYMeters = new RunningStats();
  private final RunningStats stationaryYawDeg = new RunningStats();
  private final RunningStats latencySeconds = new RunningStats();
  private final RunningStats tagDistanceMeters = new RunningStats();
  private final RunningStats fusedResidualMeters = new RunningStats();

  private int sampleCount;
  private int rejectedCount;

  /**
   * @param logRoot Prefix for all logged keys, e.g. {@code "Calibration"}.
   */
  public VisionCalibration(String logRoot) {
    this.logRoot = logRoot;
  }

  /**
   * Folds one AprilTag sighting into the calibration statistics.
   *
   * @param visionPose        Tag-derived field pose — treated as ground truth.
   * @param odometryOnlyPose  Pose from wheels and gyro alone, with no vision fused in.
   * @param fusedPose         The pose estimate the robot is actually using.
   * @param gyroYawDegrees    Raw gyro heading, same convention as the poses.
   * @param tagDistance       Distance to the nearest tag used, in metres.
   * @param latency           Age of the measurement when applied, in seconds.
   * @param chassisSpeed      Current robot speed, in m/s, used to detect standing still.
   */
  public void addSample(
      Pose2d visionPose,
      Pose2d odometryOnlyPose,
      Pose2d fusedPose,
      double gyroYawDegrees,
      double tagDistance,
      double latency,
      double chassisSpeed) {

    if (visionPose == null || odometryOnlyPose == null || fusedPose == null) {
      rejectedCount++;
      return;
    }

    sampleCount++;
    latencySeconds.add(latency);
    tagDistanceMeters.add(tagDistance);

    // How far the fused estimate sits from ground truth. Persistently large means the
    // estimator is trusting the wheels too much; near-zero and jumpy means it is trusting
    // vision too much.
    fusedResidualMeters.add(
        fusedPose.getTranslation().getDistance(visionPose.getTranslation()));

    // Gyro error against tag-measured heading, wrapped to +/-180.
    double yawError = MathUtil.inputModulus(
        visionPose.getRotation().getDegrees() - gyroYawDegrees, -180, 180);
    gyroYawErrorDeg.add(yawError);

    boolean stationary = Math.abs(chassisSpeed) < STATIONARY_SPEED_THRESHOLD;

    if (stationary) {
      // Standing still, every variation in the tag estimate is measurement noise. The
      // spread of these samples is exactly what the pose estimator's standard deviations
      // should be set to.
      stationaryXMeters.add(visionPose.getX());
      stationaryYMeters.add(visionPose.getY());
      stationaryYawDeg.add(visionPose.getRotation().getDegrees());
    } else {
      accumulatePathLengths(visionPose, odometryOnlyPose);
    }
  }

  /**
   * Accumulates matched path lengths so their ratio estimates the wheel-scale error.
   *
   * <p>Comparing path <em>lengths</em> rather than absolute positions makes this immune to a
   * wrong starting pose: only the distance travelled matters.
   */
  private void accumulatePathLengths(Pose2d visionPose, Pose2d odometryOnlyPose) {
    if (lastOdometrySample == null || lastVisionSample == null) {
      lastOdometrySample = odometryOnlyPose;
      lastVisionSample = visionPose;
      return;
    }

    double odometrySegment =
        odometryOnlyPose.getTranslation().getDistance(lastOdometrySample.getTranslation());

    if (odometrySegment < MIN_SEGMENT_METERS) {
      return; // Too short to beat the noise floor; wait for more movement.
    }

    double visionSegment =
        visionPose.getTranslation().getDistance(lastVisionSample.getTranslation());

    odometryPathMeters += odometrySegment;
    visionPathMeters += visionSegment;
    visionSegments.add(visionSegment);

    lastOdometrySample = odometryOnlyPose;
    lastVisionSample = visionPose;
  }

  /**
   * Removes the length that measurement noise adds to a path.
   *
   * <p>Summing distances between noisy positions systematically over-reports: a random walk
   * is never shorter than the straight line between its endpoints, so noise only ever adds
   * length. Left uncorrected, the wheel-scale ratio reads high — and the unit tests miss it,
   * because synthetic noise-free samples have nothing to correct.
   *
   * <p>Each measured segment is the true displacement plus the difference of two independent
   * position errors. With per-axis standard deviation sigma over two axes, that gives
   * {@code E[measured^2] = true^2 + 4 * sigma^2}, so the true length is recovered as
   * {@code sqrt(measured^2 - 4 * sigma^2)}.
   *
   * @return noise-corrected total vision path length in metres.
   */
  private double correctedVisionPathMeters() {
    double sigma = getMeasuredXyStdDevMeters();
    if (sigma <= 0 || visionSegments.isEmpty()) {
      return visionPathMeters; // Noise not measured yet; nothing defensible to subtract.
    }

    double noiseVariance = 4.0 * sigma * sigma;
    double total = 0;
    for (double measured : visionSegments) {
      total += Math.sqrt(Math.max(0.0, measured * measured - noiseVariance));
    }
    return total;
  }

  /**
   * The multiplier to apply to the configured wheel diameter.
   *
   * <p>Returns 1.0 until enough movement has been observed to be meaningful. A value of
   * 0.98 means odometry over-reports distance by 2%, so the real wheels are 2% smaller than
   * configured — multiply {@code kWheelDiameterMeters} by this number.
   *
   * @return the wheel-scale correction factor.
   */
  public double getWheelScaleEstimate() {
    if (odometryPathMeters < 1.0) {
      return 1.0;
    }
    return correctedVisionPathMeters() / odometryPathMeters;
  }

  /**
   * The same ratio without the noise correction.
   *
   * <p>Exposed for comparison: the gap between this and {@link #getWheelScaleEstimate()} is
   * how much measurement noise was inflating the answer. If the two differ substantially,
   * the camera is noisy enough that the calibration deserves a longer run.
   *
   * @return the uncorrected wheel-scale ratio.
   */
  public double getRawWheelScaleEstimate() {
    if (odometryPathMeters < 1.0) {
      return 1.0;
    }
    return visionPathMeters / odometryPathMeters;
  }

  /** @return metres of movement folded into the wheel-scale estimate so far. */
  public double getCalibrationDistanceMeters() {
    return odometryPathMeters;
  }

  /** @return true once enough distance has been driven to trust the wheel-scale estimate. */
  public boolean isWheelScaleTrustworthy() {
    return odometryPathMeters >= 10.0;
  }

  /** @return statistics for tag-measured heading minus gyro heading, in degrees. */
  public RunningStats getGyroYawError() {
    return gyroYawErrorDeg;
  }

  /**
   * Standard deviation to configure for vision x/y, in metres.
   *
   * <p>Measured from the spread of tag estimates while stationary, so this is the empirical
   * replacement for a guessed constant. Averages the x and y spreads, since the pose
   * estimator takes a single translational term.
   *
   * @return measured translational standard deviation, or 0 before enough samples.
   */
  public double getMeasuredXyStdDevMeters() {
    if (stationaryXMeters.getCount() < 10) {
      return 0;
    }
    return (stationaryXMeters.getStdDev() + stationaryYMeters.getStdDev()) / 2.0;
  }

  /**
   * @return measured rotational standard deviation in degrees, or 0 before enough samples.
   */
  public double getMeasuredYawStdDevDegrees() {
    if (stationaryYawDeg.getCount() < 10) {
      return 0;
    }
    return stationaryYawDeg.getStdDev();
  }

  /** @return statistics for measurement age, in seconds. */
  public RunningStats getLatency() {
    return latencySeconds;
  }

  /** @return statistics for distance from the fused estimate to ground truth, in metres. */
  public RunningStats getFusedResidual() {
    return fusedResidualMeters;
  }

  /** @return how many sightings have been folded in. */
  public int getSampleCount() {
    return sampleCount;
  }

  /** Publishes every calibration figure. Call once per loop. */
  public void log() {
    Logger.recordOutput(logRoot + "/Samples", sampleCount);
    Logger.recordOutput(logRoot + "/Rejected", rejectedCount);

    Logger.recordOutput(logRoot + "/WheelScale/Estimate", getWheelScaleEstimate());
    Logger.recordOutput(logRoot + "/WheelScale/EstimateRaw", getRawWheelScaleEstimate());
    Logger.recordOutput(logRoot + "/WheelScale/OdometryPathMeters", odometryPathMeters);
    Logger.recordOutput(logRoot + "/WheelScale/VisionPathMeters", visionPathMeters);
    Logger.recordOutput(logRoot + "/WheelScale/VisionPathCorrectedMeters",
        correctedVisionPathMeters());
    Logger.recordOutput(logRoot + "/WheelScale/Segments", visionSegments.size());
    Logger.recordOutput(logRoot + "/WheelScale/Trustworthy", isWheelScaleTrustworthy());

    Logger.recordOutput(logRoot + "/GyroYawError/MeanDeg", gyroYawErrorDeg.getMean());
    Logger.recordOutput(logRoot + "/GyroYawError/StdDevDeg", gyroYawErrorDeg.getStdDev());
    Logger.recordOutput(logRoot + "/GyroYawError/MaxAbsDeg",
        Math.max(Math.abs(gyroYawErrorDeg.getMin()), Math.abs(gyroYawErrorDeg.getMax())));

    Logger.recordOutput(logRoot + "/VisionNoise/StationarySamples",
        stationaryXMeters.getCount());
    Logger.recordOutput(logRoot + "/VisionNoise/MeasuredXyStdDevMeters",
        getMeasuredXyStdDevMeters());
    Logger.recordOutput(logRoot + "/VisionNoise/MeasuredYawStdDevDeg",
        getMeasuredYawStdDevDegrees());

    Logger.recordOutput(logRoot + "/Latency/MeanSeconds", latencySeconds.getMean());
    Logger.recordOutput(logRoot + "/Latency/MaxSeconds", latencySeconds.getMax());

    Logger.recordOutput(logRoot + "/TagDistance/MeanMeters", tagDistanceMeters.getMean());
    Logger.recordOutput(logRoot + "/TagDistance/MaxMeters", tagDistanceMeters.getMax());

    Logger.recordOutput(logRoot + "/FusedResidual/MeanMeters", fusedResidualMeters.getMean());
    Logger.recordOutput(logRoot + "/FusedResidual/MaxMeters", fusedResidualMeters.getMax());
  }

  /** Clears every accumulated statistic. Use between calibration runs. */
  public void reset() {
    odometryPathMeters = 0;
    visionPathMeters = 0;
    visionSegments.clear();
    lastOdometrySample = null;
    lastVisionSample = null;
    sampleCount = 0;
    rejectedCount = 0;

    gyroYawErrorDeg.reset();
    stationaryXMeters.reset();
    stationaryYMeters.reset();
    stationaryYawDeg.reset();
    latencySeconds.reset();
    tagDistanceMeters.reset();
    fusedResidualMeters.reset();
  }
}
