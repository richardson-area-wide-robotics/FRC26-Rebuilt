package frc.robot.common.components.diagnostics;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/**
 * Continuously checks that the robot is behaving the way the code says it should, and
 * records the answer to AdvantageKit.
 *
 * <p>The point of this class is to turn "the robot feels wrong" into a specific, timestamped
 * claim you can find in a log. Each {@link Expectation} is a named invariant — "when the
 * driver pushes the stick, the modules are commanded to move", "the gyro stays connected",
 * "the shooter reaches its setpoint when it is asked to spin" — evaluated every robot loop.
 *
 * <p>Every expectation publishes three signals under {@code Expectations/<name>/}:
 * <ul>
 *   <li>{@code OK} — is the invariant holding right now</li>
 *   <li>{@code Violations} — how many times it has been broken this power cycle</li>
 *   <li>{@code FirstViolationTimestamp} — FPGA time of the first break, or −1</li>
 * </ul>
 * plus rolled-up {@code Expectations/AllOK} and {@code Expectations/TotalViolations}, so a
 * single boolean on the dashboard tells a driver whether to trust the robot.
 *
 * <p>Expectations are deliberately cheap and side-effect free. They never stop the robot;
 * they only observe. Anything that must actually intervene belongs in a subsystem.
 */
public final class ExpectationMonitor {

  /** A single named invariant, with debounce so transient blips don't register. */
  public static final class Expectation {
    private final String name;
    private final String description;
    private final BooleanSupplier holds;
    private final int requiredConsecutiveFailures;

    private int consecutiveFailures;
    private int violations;
    private double firstViolationTimestamp = -1;
    private boolean currentlyOk = true;

    private Expectation(
        String name, String description, BooleanSupplier holds, int requiredConsecutiveFailures) {
      this.name = name;
      this.description = description;
      this.holds = holds;
      this.requiredConsecutiveFailures = Math.max(1, requiredConsecutiveFailures);
    }

    private void evaluate() {
      boolean ok;
      try {
        ok = holds.getAsBoolean();
      } catch (RuntimeException e) {
        // An expectation that throws is itself a failure, and must never take the robot
        // down — this monitor runs inside robotPeriodic().
        ok = false;
      }

      if (ok) {
        consecutiveFailures = 0;
        currentlyOk = true;
      } else {
        consecutiveFailures++;
        if (consecutiveFailures >= requiredConsecutiveFailures) {
          if (currentlyOk) {
            // Only count a transition into failure, not every loop spent failing.
            violations++;
            if (firstViolationTimestamp < 0) {
              firstViolationTimestamp = Timer.getFPGATimestamp();
            }
          }
          currentlyOk = false;
        }
      }
    }

    private void log() {
      String root = "Expectations/" + name;
      Logger.recordOutput(root + "/OK", currentlyOk);
      Logger.recordOutput(root + "/Violations", violations);
      Logger.recordOutput(root + "/FirstViolationTimestamp", firstViolationTimestamp);
    }

    public String getName() {
      return name;
    }

    public String getDescription() {
      return description;
    }

    public boolean isOk() {
      return currentlyOk;
    }

    public int getViolations() {
      return violations;
    }

    public double getFirstViolationTimestamp() {
      return firstViolationTimestamp;
    }

    private void reset() {
      consecutiveFailures = 0;
      violations = 0;
      firstViolationTimestamp = -1;
      currentlyOk = true;
    }
  }

  private static ExpectationMonitor instance;

  private final List<Expectation> expectations = new ArrayList<>();

  private ExpectationMonitor() {
  }

  public static synchronized ExpectationMonitor getInstance() {
    if (instance == null) {
      instance = new ExpectationMonitor();
    }
    return instance;
  }

  /**
   * Registers an invariant that must hold on every loop once it starts failing
   * consecutively.
   *
   * @param name                        Short stable name; becomes the log key.
   * @param description                 Human-readable statement of what should be true.
   * @param holds                       Returns true while the invariant holds.
   * @param requiredConsecutiveFailures How many loops in a row must fail before it counts,
   *                                    to absorb single-loop sensor blips.
   * @return the registered expectation, for direct inspection in tests.
   */
  public Expectation register(
      String name, String description, BooleanSupplier holds, int requiredConsecutiveFailures) {
    Expectation expectation =
        new Expectation(name, description, holds, requiredConsecutiveFailures);
    expectations.add(expectation);
    return expectation;
  }

  /** Registers an invariant that counts after 3 consecutive failing loops (~60 ms). */
  public Expectation register(String name, String description, BooleanSupplier holds) {
    return register(name, description, holds, 3);
  }

  /** Evaluates and logs every registered expectation. Call once per robot loop. */
  public void update() {
    int totalViolations = 0;
    boolean allOk = true;

    for (Expectation expectation : expectations) {
      expectation.evaluate();
      expectation.log();
      totalViolations += expectation.getViolations();
      allOk = allOk && expectation.isOk();
    }

    Logger.recordOutput("Expectations/AllOK", allOk);
    Logger.recordOutput("Expectations/TotalViolations", totalViolations);
    Logger.recordOutput("Expectations/Count", expectations.size());
  }

  /** @return true when every registered expectation currently holds. */
  public boolean allOk() {
    for (Expectation expectation : expectations) {
      if (!expectation.isOk()) {
        return false;
      }
    }
    return true;
  }

  /** @return every expectation currently broken, for reporting. */
  public List<Expectation> getViolations() {
    List<Expectation> broken = new ArrayList<>();
    for (Expectation expectation : expectations) {
      if (!expectation.isOk()) {
        broken.add(expectation);
      }
    }
    return broken;
  }

  /** @return an unmodifiable view of every registered expectation. */
  public List<Expectation> getExpectations() {
    return List.copyOf(expectations);
  }

  /** Clears violation history without unregistering anything. */
  public void resetCounters() {
    for (Expectation expectation : expectations) {
      expectation.reset();
    }
  }

  /**
   * Removes every registered expectation. Intended for tests, which need a clean monitor
   * between cases; the robot registers its set once at startup.
   */
  public void clear() {
    expectations.clear();
  }
}
