package frc.robot.common.components.diagnostics;

import java.util.function.DoubleConsumer;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A constant you can change from the dashboard while the robot is running, then read back
 * out of the log to decide what the value should permanently become.
 *
 * <p>This is the mechanism for tuning the robot's constants against real data instead of
 * guesswork. Wrap a number in a {@code TunableNumber}, set {@link #TUNING_ENABLED} true,
 * drive the robot while adjusting the value in AdvantageScope or Shuffleboard, and every
 * value you tried is recorded alongside the resulting behaviour. When you settle on a
 * number, paste it back into the constants file and turn tuning off.
 *
 * <p>With {@link #TUNING_ENABLED} false the class is inert: it returns the compiled-in
 * default and publishes nothing extra, so leaving {@code TunableNumber}s in competition code
 * costs nothing and cannot be changed accidentally from the pits.
 */
public class TunableNumber {

  /**
   * Master switch for live tuning.
   *
   * <p>Keep this {@code false} for competition. When false, every {@code TunableNumber}
   * returns its hard-coded default and ignores the dashboard entirely — so a stray edit in
   * the pits can't change robot behaviour mid-event.
   */
  public static final boolean TUNING_ENABLED = false;

  private static final String TABLE_ROOT = "Tuning/";

  private final String key;
  private final double defaultValue;
  private final LoggedNetworkNumber dashboardValue;

  private double lastValue;

  /**
   * @param key          Stable name; appears on the dashboard under {@code Tuning/}.
   * @param defaultValue The value used in competition, and the starting point for tuning.
   */
  public TunableNumber(String key, double defaultValue) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.lastValue = defaultValue;
    this.dashboardValue =
        TUNING_ENABLED ? new LoggedNetworkNumber(TABLE_ROOT + key, defaultValue) : null;
  }

  /** @return the dashboard value while tuning, otherwise the compiled-in default. */
  public double get() {
    if (dashboardValue == null) {
      return defaultValue;
    }
    lastValue = dashboardValue.get();
    return lastValue;
  }

  /** @return the value this number was compiled with, ignoring any dashboard override. */
  public double getDefault() {
    return defaultValue;
  }

  /** @return the dashboard key this number is published under. */
  public String getKey() {
    return key;
  }

  /**
   * Calls {@code onChange} only when the value has actually moved since the last check.
   *
   * <p>Use this to avoid re-sending motor controller configuration every loop while tuning:
   * SPARK configuration writes are slow and flood the CAN bus if done unconditionally.
   *
   * @param onChange Invoked with the new value when it differs from the previous one.
   */
  public void ifChanged(DoubleConsumer onChange) {
    if (dashboardValue == null) {
      return;
    }
    double previous = lastValue;
    double current = get();
    if (Double.compare(previous, current) != 0) {
      Logger.recordOutput(TABLE_ROOT + key + "/Changed", current);
      onChange.accept(current);
    }
  }
}
