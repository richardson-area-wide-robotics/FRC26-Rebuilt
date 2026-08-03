package frc.robot.common.components.diagnostics;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;

/**
 * The maths behind the drivetrain auto-calibrator.
 *
 * <p>Deliberately free of hardware, commands and WPILib subsystems so every formula here can
 * be unit tested against synthetic data with a known answer. The routines that actually move
 * the robot live in {@code DriveAutoCalibrator}; this class only turns their measurements into
 * constants.
 *
 * <p>Each method documents which sensors it trusts and why. The general principle: AprilTags
 * supply absolute truth at low rate, encoders supply precise relative motion at high rate but
 * with uncertain scale, and the gyro supplies high-rate heading with a scale error of its own.
 * Calibration means using the absolute source to pin the scale of the relative ones.
 */
public final class DriveCharacterization {

  private DriveCharacterization() {
  }

  // ---------------------------------------------------------------------------------------
  // Feedforward: kS and kV
  // ---------------------------------------------------------------------------------------

  /** Result of a feedforward sweep. */
  public record Feedforward(double kS, double kV, double rSquared, int samples) {

    /** @return true when the fit explains essentially all the variance. */
    public boolean isTrustworthy() {
      return samples >= 4 && rSquared > 0.95 && kV > 0;
    }
  }

  /**
   * Accumulates (voltage, steady-state velocity) pairs and fits {@code V = kS + kV * v}.
   *
   * <p>Velocity comes from the drive encoders, not from AprilTags: encoders update at ~50 Hz
   * with far less noise than a tag solve, and for this fit only the <em>scale</em> needs to be
   * right — which is what the wheel-scale calibration corrects. Run that first.
   *
   * <p>kS is the intercept: the voltage needed before the wheels move at all, dominated by
   * static friction. kV is the slope: volts per metre per second.
   */
  public static final class FeedforwardFit {
    private final List<double[]> samples = new ArrayList<>();

    /**
     * @param volts             Applied voltage.
     * @param velocityMetersSec Measured steady-state velocity.
     */
    public void add(double volts, double velocityMetersSec) {
      // Points below the friction threshold carry no information about the slope and drag
      // the intercept around, so ignore anything that did not actually move.
      if (Math.abs(velocityMetersSec) < 1e-3) {
        return;
      }
      samples.add(new double[] {velocityMetersSec, volts});
    }

    public int size() {
      return samples.size();
    }

    /**
     * Least-squares fit of voltage against velocity.
     *
     * @return the fitted feedforward, or a zero fit with fewer than two usable samples.
     */
    public Feedforward fit() {
      int n = samples.size();
      if (n < 2) {
        return new Feedforward(0, 0, 0, n);
      }

      double sumX = 0;
      double sumY = 0;
      for (double[] s : samples) {
        sumX += s[0];
        sumY += s[1];
      }
      double meanX = sumX / n;
      double meanY = sumY / n;

      double sxx = 0;
      double sxy = 0;
      for (double[] s : samples) {
        double dx = s[0] - meanX;
        sxx += dx * dx;
        sxy += dx * (s[1] - meanY);
      }

      if (sxx <= 0) {
        return new Feedforward(0, 0, 0, n);
      }

      double kV = sxy / sxx;
      double kS = meanY - kV * meanX;

      // Coefficient of determination, so a bad sweep is visibly bad rather than silently
      // producing a confident-looking number.
      double ssTot = 0;
      double ssRes = 0;
      for (double[] s : samples) {
        double predicted = kS + kV * s[0];
        ssTot += Math.pow(s[1] - meanY, 2);
        ssRes += Math.pow(s[1] - predicted, 2);
      }
      double rSquared = ssTot <= 0 ? 0 : 1.0 - ssRes / ssTot;

      return new Feedforward(kS, kV, rSquared, n);
    }

    public void reset() {
      samples.clear();
    }
  }

  /**
   * How much four modules disagree about their own feedforward.
   *
   * @param meanKv          Mean kV across the modules that produced a usable fit.
   * @param minKv           Lowest kV seen.
   * @param maxKv           Highest kV seen.
   * @param spreadPercent   Peak-to-peak spread as a percentage of the mean.
   * @param worstModule     Index of the module furthest from the mean, or −1 if none.
   * @param usableFits      How many modules produced a trustworthy fit.
   */
  public record ModuleVariance(
      double meanKv, double minKv, double maxKv, double spreadPercent,
      int worstModule, int usableFits) {

    /**
     * Whether the spread is small enough to treat the drivetrain as four identical motors.
     *
     * <p>Published free speed is a typical figure, not a guarantee: individual motors vary, and
     * REV's own 5676 RPM is an empirical average. A few percent of spread is normal and can be
     * absorbed by the velocity loop. Much more than that and one corner is materially weaker
     * than the others, which pulls the robot off a straight line — the same cross-track error
     * the 1 inch budget is fighting.
     *
     * @return true when the modules are close enough to share one kV.
     */
    public boolean isUniform() {
      return usableFits >= 3 && spreadPercent < 8.0;
    }
  }

  /**
   * Summarises per-module feedforward fits so motor-to-motor variance is visible.
   *
   * <p>Averaging voltage and velocity across all four modules before fitting produces one tidy
   * kV and hides the thing worth knowing. Fitting each module separately and comparing is what
   * reveals a weak or mis-geared corner.
   *
   * @param fits One fit per module, in FL, FR, RL, RR order. Entries may be untrustworthy.
   * @return the variance summary.
   */
  public static ModuleVariance summariseModuleVariance(Feedforward[] fits) {
    double sum = 0;
    double min = Double.POSITIVE_INFINITY;
    double max = Double.NEGATIVE_INFINITY;
    int usable = 0;

    for (Feedforward fit : fits) {
      if (fit == null || !fit.isTrustworthy()) {
        continue;
      }
      usable++;
      sum += fit.kV();
      min = Math.min(min, fit.kV());
      max = Math.max(max, fit.kV());
    }

    if (usable == 0) {
      return new ModuleVariance(0, 0, 0, 0, -1, 0);
    }

    double mean = sum / usable;
    double spreadPercent = mean > 0 ? (max - min) / mean * 100.0 : 0;

    // Which module sits furthest from the mean — the one to look at first.
    int worst = -1;
    double worstDeviation = -1;
    for (int i = 0; i < fits.length; i++) {
      if (fits[i] == null || !fits[i].isTrustworthy()) {
        continue;
      }
      double deviation = Math.abs(fits[i].kV() - mean);
      if (deviation > worstDeviation) {
        worstDeviation = deviation;
        worst = i;
      }
    }

    return new ModuleVariance(mean, min, max, spreadPercent, worst, usable);
  }

  // ---------------------------------------------------------------------------------------
  // Scale factors
  // ---------------------------------------------------------------------------------------

  /**
   * Wheel-diameter correction from a straight run.
   *
   * <p>The multiplier for {@code kWheelDiameterMeters}. Under 1.0 means odometry over-reports
   * distance, which is the normal direction: the effective rolling diameter of a compressed
   * tread is smaller than nominal.
   *
   * @param tagDistanceMeters      Distance travelled according to AprilTags.
   * @param odometryDistanceMeters Distance travelled according to the wheel encoders.
   * @return the correction factor, or 1.0 if the run was too short to mean anything.
   */
  public static double wheelScale(double tagDistanceMeters, double odometryDistanceMeters) {
    if (odometryDistanceMeters < 0.5 || tagDistanceMeters <= 0) {
      return 1.0;
    }
    return tagDistanceMeters / odometryDistanceMeters;
  }

  /**
   * Gyro scale correction from a rotation.
   *
   * <p>Multiply gyro readings by this. Rotating several full turns rather than one makes the
   * scale error large compared with the tag measurement noise.
   *
   * @param tagYawDeltaDegrees  Total heading change according to AprilTags, unwrapped.
   * @param gyroYawDeltaDegrees Total heading change according to the gyro, unwrapped.
   * @return the correction factor, or 1.0 if the rotation was too small.
   */
  public static double gyroScale(double tagYawDeltaDegrees, double gyroYawDeltaDegrees) {
    if (Math.abs(gyroYawDeltaDegrees) < 90.0) {
      return 1.0;
    }
    return tagYawDeltaDegrees / gyroYawDeltaDegrees;
  }

  /**
   * Effective drive radius from spinning in place.
   *
   * <p>For a swerve chassis rotating about its centre, each module travels at
   * {@code v = omega * r}, where r is the distance from centre to module. Measuring it rather
   * than deriving it from the frame dimensions captures scrub and module toe, which is why the
   * effective value is normally a little larger than the geometric one.
   *
   * @param moduleSpeedMetersSec Average absolute module speed.
   * @param omegaRadPerSec       Measured rotation rate.
   * @return effective radius in metres, or 0 if the robot was barely turning.
   */
  public static double effectiveDriveRadius(
      double moduleSpeedMetersSec, double omegaRadPerSec) {
    if (Math.abs(omegaRadPerSec) < 0.2) {
      return 0;
    }
    return Math.abs(moduleSpeedMetersSec / omegaRadPerSec);
  }

  /**
   * Geometric drive radius implied by the configured frame dimensions.
   *
   * @param trackWidthMeters Left-to-right wheel spacing.
   * @param wheelBaseMeters  Front-to-back wheel spacing.
   * @return distance from chassis centre to a module.
   */
  public static double geometricDriveRadius(double trackWidthMeters, double wheelBaseMeters) {
    return Math.hypot(trackWidthMeters / 2.0, wheelBaseMeters / 2.0);
  }

  // ---------------------------------------------------------------------------------------
  // Steering alignment — the term that can eat a 1 inch budget on its own
  // ---------------------------------------------------------------------------------------

  /**
   * Common-mode steering misalignment, in degrees.
   *
   * <p>If every module sits at the same small angle away from where it was commanded, the
   * robot tracks a straight line that is rotated from the one requested. The absolute
   * encoders are specified to +/-0.5 degrees, and 0.5 degrees over 10 feet is 26 mm of
   * cross-track error — enough to consume a 1 inch budget by itself. So this is worth
   * measuring rather than trusting.
   *
   * <p>Measured by driving straight and comparing the direction the robot was asked to travel
   * with the direction it actually travelled according to AprilTags.
   *
   * @param commandedHeadingDegrees Field-relative direction commanded.
   * @param actualTravelDegrees     Field-relative direction actually travelled, from tags.
   * @return the signed offset to subtract from the module angular offsets, in degrees.
   */
  public static double commonModeSteerOffsetDegrees(
      double commandedHeadingDegrees, double actualTravelDegrees) {
    return MathUtil.inputModulus(actualTravelDegrees - commandedHeadingDegrees, -180, 180);
  }

  /**
   * Cross-track error a given steering misalignment produces over a given distance.
   *
   * <p>Used to report the misalignment in the units the requirement is written in.
   *
   * @param offsetDegrees   Steering misalignment.
   * @param distanceMeters  Distance driven.
   * @return lateral error in metres.
   */
  public static double crossTrackErrorFromSteerOffset(
      double offsetDegrees, double distanceMeters) {
    return distanceMeters * Math.sin(Math.toRadians(offsetDegrees));
  }

  // ---------------------------------------------------------------------------------------
  // Closed-loop gain proposal
  // ---------------------------------------------------------------------------------------

  /**
   * Scales a proportional gain to hit a target steady-state error.
   *
   * <p>For proportional control, steady-state error is inversely proportional to loop gain, so
   * if the current gain leaves {@code measuredError} and the goal is {@code targetError}, the
   * gain should scale by their ratio.
   *
   * <p>This is a <b>proposal, not a tuned result</b>: it ignores dynamics, so it can suggest a
   * gain that oscillates. The change is clamped to a factor of 4 per iteration so a single
   * noisy measurement cannot produce a wild value, and it should be applied, re-measured, and
   * repeated rather than trusted in one shot.
   *
   * @param currentGain   Gain currently in use.
   * @param measuredError Steady-state error observed with that gain.
   * @param targetError   Steady-state error wanted.
   * @return the proposed gain.
   */
  public static double proposeProportionalGain(
      double currentGain, double measuredError, double targetError) {
    if (currentGain <= 0 || targetError <= 0 || measuredError <= 0) {
      return currentGain;
    }
    double ratio = MathUtil.clamp(measuredError / targetError, 0.25, 4.0);
    return currentGain * ratio;
  }

  // ---------------------------------------------------------------------------------------
  // Acceptance
  // ---------------------------------------------------------------------------------------

  /** Outcome of a straight-line accuracy run. */
  public record StraightRunResult(
      double commandedDistanceMeters,
      double alongTrackErrorMeters,
      double crossTrackErrorMeters,
      double headingErrorDegrees) {

    /** @return total positional error in metres. */
    public double totalErrorMeters() {
      return Math.hypot(alongTrackErrorMeters, crossTrackErrorMeters);
    }

    /**
     * @param toleranceMeters Allowed total error.
     * @return true when the run met the requirement.
     */
    public boolean meetsTolerance(double toleranceMeters) {
      return totalErrorMeters() <= toleranceMeters;
    }
  }

  /**
   * Decomposes a straight-run result into along-track and cross-track error.
   *
   * <p>The two failure modes have different causes and different fixes: along-track error is
   * wheel scale, cross-track error is heading or steering alignment. Reporting a single
   * distance number would hide which one is at fault.
   *
   * @param commandedDistanceMeters How far the robot was asked to travel.
   * @param commandedHeadingDegrees The direction it was asked to travel.
   * @param actualDxMeters          Actual displacement in field x, from AprilTags.
   * @param actualDyMeters          Actual displacement in field y, from AprilTags.
   * @param finalHeadingErrorDeg    Heading error at the end of the run.
   * @return the decomposed result.
   */
  public static StraightRunResult analyseStraightRun(
      double commandedDistanceMeters,
      double commandedHeadingDegrees,
      double actualDxMeters,
      double actualDyMeters,
      double finalHeadingErrorDeg) {

    double heading = Math.toRadians(commandedHeadingDegrees);
    // Project actual displacement onto the commanded direction and its perpendicular.
    double along = actualDxMeters * Math.cos(heading) + actualDyMeters * Math.sin(heading);
    double cross = -actualDxMeters * Math.sin(heading) + actualDyMeters * Math.cos(heading);

    return new StraightRunResult(
        commandedDistanceMeters,
        along - commandedDistanceMeters,
        cross,
        finalHeadingErrorDeg);
  }
}
