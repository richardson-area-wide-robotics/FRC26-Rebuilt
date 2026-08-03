package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.common.components.diagnostics.ExpectationMonitor.Expectation;
import frc.robot.testutil.HalFixture;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

/**
 * Tests for the expectation monitor, the mechanism that turns "the robot feels wrong" into a
 * timestamped, named claim in the log.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class ExpectationMonitorTest {

  private ExpectationMonitor monitor;

  @BeforeAll
  void setUpClass() {
    HalFixture.initialize();
  }

  @BeforeEach
  void setUp() {
    monitor = ExpectationMonitor.getInstance();
    monitor.clear();
  }

  @Test
  @DisplayName("An invariant that holds records no violation")
  void holdingInvariantIsClean() {
    Expectation always = monitor.register("AlwaysTrue", "Always holds", () -> true, 1);

    for (int i = 0; i < 10; i++) {
      monitor.update();
    }

    assertTrue(always.isOk());
    assertEquals(0, always.getViolations());
    assertTrue(monitor.allOk());
    assertEquals(-1, always.getFirstViolationTimestamp(), 1e-9);
  }

  @Test
  @DisplayName("A single failing loop is absorbed by the debounce")
  void singleBlipIsAbsorbed() {
    boolean[] healthy = {true};
    Expectation flaky = monitor.register("Flaky", "Debounced", () -> healthy[0], 3);

    monitor.update();
    healthy[0] = false;
    monitor.update(); // 1st consecutive failure — below the threshold of 3
    assertTrue(flaky.isOk(), "One bad loop must not register as a violation");

    healthy[0] = true;
    monitor.update();
    assertEquals(0, flaky.getViolations());
  }

  @Test
  @DisplayName("Sustained failure past the debounce registers exactly one violation")
  void sustainedFailureCountsOnce() {
    boolean[] healthy = {true};
    Expectation sustained = monitor.register("Sustained", "Counts once", () -> healthy[0], 3);

    monitor.update();
    healthy[0] = false;
    for (int i = 0; i < 20; i++) {
      monitor.update();
    }

    assertFalse(sustained.isOk());
    assertEquals(1, sustained.getViolations(),
        "Twenty failing loops is one fault, not twenty");
    assertTrue(sustained.getFirstViolationTimestamp() >= 0,
        "First violation must be timestamped");
    assertFalse(monitor.allOk());
    assertEquals(1, monitor.getViolations().size());
  }

  @Test
  @DisplayName("Recovering and failing again counts as two violations")
  void recoveryThenFailureCountsTwice() {
    boolean[] healthy = {true};
    Expectation intermittent = monitor.register("Intermittent", "Counts each fault",
        () -> healthy[0], 1);

    healthy[0] = false;
    monitor.update();
    healthy[0] = true;
    monitor.update();
    healthy[0] = false;
    monitor.update();

    assertEquals(2, intermittent.getViolations());
  }

  @Test
  @DisplayName("An invariant that throws is a failure, not a crash")
  void throwingInvariantIsContained() {
    Expectation exploding = monitor.register("Exploding", "Throws every loop",
        () -> {
          throw new IllegalStateException("sensor unplugged");
        },
        1);

    // This must not propagate: the monitor runs inside robotPeriodic().
    monitor.update();

    assertFalse(exploding.isOk(), "A throwing check must be treated as failing");
    assertEquals(1, exploding.getViolations());
  }

  @Test
  @DisplayName("Counters reset without unregistering")
  void resetCountersKeepsRegistrations() {
    Expectation failing = monitor.register("Failing", "Always fails", () -> false, 1);
    monitor.update();
    assertEquals(1, failing.getViolations());

    monitor.resetCounters();

    assertEquals(0, failing.getViolations());
    assertTrue(failing.isOk());
    assertEquals(-1, failing.getFirstViolationTimestamp(), 1e-9);
    assertEquals(1, monitor.getExpectations().size(), "Reset must not unregister");
  }

  @Test
  @DisplayName("Multiple expectations are tracked independently")
  void expectationsAreIndependent() {
    Expectation good = monitor.register("Good", "Holds", () -> true, 1);
    Expectation bad = monitor.register("Bad", "Fails", () -> false, 1);

    monitor.update();

    assertTrue(good.isOk());
    assertFalse(bad.isOk());
    assertEquals(2, monitor.getExpectations().size());
    assertEquals(1, monitor.getViolations().size());
    assertEquals("Bad", monitor.getViolations().get(0).getName());
  }

  @Test
  @DisplayName("Registered metadata is preserved for reporting")
  void metadataIsPreserved() {
    Expectation e = monitor.register("Named", "A human readable description", () -> true, 1);
    assertEquals("Named", e.getName());
    assertEquals("A human readable description", e.getDescription());
  }

  @Test
  @DisplayName("Default registration uses a three-loop debounce")
  void defaultDebounceIsThree() {
    Expectation e = monitor.register("Defaulted", "Uses the default debounce", () -> false);

    monitor.update();
    monitor.update();
    assertTrue(e.isOk(), "Two failing loops should still be within the default debounce");

    monitor.update();
    assertFalse(e.isOk(), "The third consecutive failure should register");
  }
}
