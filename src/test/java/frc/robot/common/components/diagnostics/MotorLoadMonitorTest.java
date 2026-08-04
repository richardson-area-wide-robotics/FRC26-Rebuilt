package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.common.components.diagnostics.MotorLoadMonitor.LoadState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Tests for load-based game piece and jam detection.
 *
 * <p>The interesting cases are all about ambiguity. A roller drawing 25 A might be ingesting a piece
 * or might be jammed; it might also just be running on a fresh battery. Getting those wrong in
 * either direction is costly: miss pieces and the driver stops trusting the indicator, cry jam
 * during normal operation and the robot starts jostling itself mid-match for no reason.
 */
class MotorLoadMonitorTest {

  /** Unloaded roller current, in amps. */
  private static final double IDLE_AMPS = 8.0;

  /** Expected unloaded roller speed, in RPM. */
  private static final double FULL_SPEED = 5000;

  private MotorLoadMonitor monitor;

  @BeforeEach
  void setUp() {
    // 10 A above baseline means a piece; below 30% of full speed means stuck; 10 loops to confirm.
    monitor = new MotorLoadMonitor("Test/Rollers", 10.0, FULL_SPEED, 0.30, 10);
  }

  /** Runs the mechanism unloaded long enough to teach it its baseline. */
  private void establishBaseline() {
    for (int i = 0; i < 60; i++) {
      monitor.update(IDLE_AMPS, FULL_SPEED, true);
    }
  }

  @Nested
  @DisplayName("Baseline learning")
  class Baseline {

    @Test
    @DisplayName("Detection stays silent until a baseline exists")
    void noDetectionBeforeBaseline() {
      // A big spike on the very first loop must not be believed — there is nothing to compare to.
      monitor.update(40.0, FULL_SPEED, true);

      assertFalse(monitor.isBaselineEstablished());
      assertFalse(monitor.isDoingWork(),
          "Reporting a piece before the baseline is known is guessing");
    }

    @Test
    @DisplayName("Running unloaded establishes the baseline near the idle current")
    void baselineConvergesOnIdle() {
      establishBaseline();

      assertTrue(monitor.isBaselineEstablished());
      assertEquals(IDLE_AMPS, monitor.getBaselineCurrent(), 1.0);
      assertEquals(LoadState.RUNNING_EMPTY, monitor.getState());
    }

    @Test
    @DisplayName("A game piece does not get absorbed into the baseline")
    void loadedCurrentDoesNotPoisonBaseline() {
      establishBaseline();
      double before = monitor.getBaselineCurrent();

      // Sustained loaded current — if this were folded into the baseline, the monitor would learn
      // that loaded is normal and stop detecting pieces entirely.
      for (int i = 0; i < 100; i++) {
        monitor.update(IDLE_AMPS + 20, FULL_SPEED * 0.9, true);
      }

      assertEquals(before, monitor.getBaselineCurrent(), 1.0,
          "The baseline must only learn while unloaded, or it erases the signal it exists to find");
      assertTrue(monitor.isDoingWork(), "And it should still be reporting the piece");
    }

    @Test
    @DisplayName("A stopped mechanism does not drag the baseline towards zero")
    void idleDoesNotPoisonBaseline() {
      establishBaseline();
      double before = monitor.getBaselineCurrent();

      for (int i = 0; i < 100; i++) {
        monitor.update(0.0, 0.0, false);
      }

      assertEquals(before, monitor.getBaselineCurrent(), 0.5,
          "Folding zeros in while stopped would make every later run look loaded");
      assertEquals(LoadState.IDLE, monitor.getState());
    }

    @Test
    @DisplayName("The baseline follows a slow drift, such as battery sag")
    void baselineTracksSlowDrift() {
      establishBaseline();

      // Battery sags over a match, so unloaded current creeps up. The baseline should follow,
      // otherwise the drift alone eventually reads as a permanent game piece.
      for (int i = 0; i < 300; i++) {
        monitor.update(IDLE_AMPS + 4, FULL_SPEED, true);
      }

      assertEquals(IDLE_AMPS + 4, monitor.getBaselineCurrent(), 1.5);
      assertFalse(monitor.isDoingWork(),
          "A slow 4 A drift is not a game piece and must not read as one");
    }
  }

  @Nested
  @DisplayName("Telling work from a jam")
  class WorkVersusJam {

    @Test
    @DisplayName("High current with healthy speed is work, not a jam")
    void highCurrentHealthySpeedIsWork() {
      establishBaseline();

      // The defining case: a piece going through loads the roller but it keeps turning.
      for (int i = 0; i < 30; i++) {
        monitor.update(IDLE_AMPS + 15, FULL_SPEED * 0.85, true);
      }

      assertEquals(LoadState.DOING_WORK, monitor.getState());
      assertTrue(monitor.isDoingWork());
      assertFalse(monitor.isJammed(),
          "A loaded roller that is still turning is doing its job, not jammed");
    }

    @Test
    @DisplayName("High current with collapsed speed is a jam")
    void highCurrentLowSpeedIsJam() {
      establishBaseline();

      for (int i = 0; i < 30; i++) {
        monitor.update(IDLE_AMPS + 15, FULL_SPEED * 0.10, true);
      }

      assertTrue(monitor.isJammed(),
          "Loaded but barely turning is the signature of a stuck piece");
    }

    @Test
    @DisplayName("Near-zero speed is reported as a stall, distinct from a jam")
    void zeroSpeedIsStall() {
      establishBaseline();

      for (int i = 0; i < 30; i++) {
        monitor.update(IDLE_AMPS + 25, 0.0, true);
      }

      assertEquals(LoadState.STALLED, monitor.getState(),
          "A hard stop deserves backing off rather than jostling harder");
      assertTrue(monitor.isJammed(), "Still needs clearing either way");
    }

    @Test
    @DisplayName("Low speed at normal current is not a jam")
    void lowSpeedAloneIsNotJam() {
      establishBaseline();

      // Spinning down after being commanded off, or simply geared slow: no excess current, so
      // nothing is stuck.
      for (int i = 0; i < 30; i++) {
        monitor.update(IDLE_AMPS, FULL_SPEED * 0.1, true);
      }

      assertFalse(monitor.isJammed(),
          "Without elevated current there is no evidence of anything being stuck");
    }

    @Test
    @DisplayName("A jam must persist before it is called")
    void jamNeedsPersistence() {
      establishBaseline();

      // Momentary bog-down, five loops, below the ten-loop confirmation.
      for (int i = 0; i < 5; i++) {
        monitor.update(IDLE_AMPS + 15, FULL_SPEED * 0.1, true);
      }

      assertFalse(monitor.isJammed(),
          "A brief bog-down as a piece seats is normal and must not trigger jostling");
    }

    @Test
    @DisplayName("Recovering resets the jam counter, so intermittent load never accumulates")
    void recoveryResetsJamCounter() {
      establishBaseline();

      // Alternate stuck and free, never nine consecutive stuck loops.
      for (int cycle = 0; cycle < 10; cycle++) {
        for (int i = 0; i < 8; i++) {
          monitor.update(IDLE_AMPS + 15, FULL_SPEED * 0.1, true);
        }
        monitor.update(IDLE_AMPS + 15, FULL_SPEED * 0.9, true);
      }

      assertFalse(monitor.isJammed(),
          "A mechanism that keeps recovering is working hard, not jammed");
    }
  }

  @Nested
  @DisplayName("Noise rejection")
  class NoiseRejection {

    @Test
    @DisplayName("A single-sample current spike does not read as a piece")
    void singleSampleSpikeIsFiltered() {
      establishBaseline();

      // SPARK current readings regularly double for one sample. The moving average absorbs it.
      monitor.update(IDLE_AMPS + 40, FULL_SPEED, true);

      assertFalse(monitor.isDoingWork(),
          "One noisy sample must not be enough; a 5-sample average sees straight through it");
    }

    @Test
    @DisplayName("Current just under the threshold does not trigger")
    void justUnderThresholdIsQuiet() {
      establishBaseline();

      for (int i = 0; i < 30; i++) {
        monitor.update(IDLE_AMPS + 9.0, FULL_SPEED, true);
      }

      assertFalse(monitor.isDoingWork(),
          "9 A of excess against a 10 A threshold should stay quiet");
    }

    @Test
    @DisplayName("Reset forgets everything, for a mechanism or game-piece change")
    void resetClearsState() {
      establishBaseline();
      assertTrue(monitor.isBaselineEstablished());

      monitor.reset();

      assertFalse(monitor.isBaselineEstablished());
      assertEquals(0.0, monitor.getBaselineCurrent(), 1e-9);
      assertEquals(LoadState.IDLE, monitor.getState());
    }
  }

  @Nested
  @DisplayName("Piece counting")
  class Counting {

    private GamePieceCounter counter;

    @BeforeEach
    void setUpCounter() {
      // 3 loops of sustained work to count, then 25 loops before another can be counted.
      counter = new GamePieceCounter("Test", monitor, 3, 25);
    }

    /** Feeds one piece through: loaded for a while, then clear. */
    private void onePieceThroughRollers() {
      for (int i = 0; i < 15; i++) {
        monitor.update(IDLE_AMPS + 15, FULL_SPEED * 0.85, true);
        counter.update();
      }
      for (int i = 0; i < 30; i++) {
        monitor.update(IDLE_AMPS, FULL_SPEED, true);
        counter.update();
      }
    }

    @Test
    @DisplayName("One piece counts once, not once per loop")
    void onePieceCountsOnce() {
      establishBaseline();
      onePieceThroughRollers();

      assertEquals(1, counter.getCount(),
          "Counting rising edges without a sustain-and-refractory guard runs away instantly");
    }

    @Test
    @DisplayName("Two separated pieces count twice")
    void twoPiecesCountTwice() {
      establishBaseline();
      onePieceThroughRollers();
      onePieceThroughRollers();

      assertEquals(2, counter.getCount());
    }

    @Test
    @DisplayName("A brief load does not count")
    void briefLoadDoesNotCount() {
      establishBaseline();

      // Two loops of load, below the three-loop sustain.
      for (int i = 0; i < 2; i++) {
        monitor.update(IDLE_AMPS + 15, FULL_SPEED * 0.85, true);
        counter.update();
      }

      assertEquals(0, counter.getCount(),
          "A roller biting on nothing should not be recorded as a game piece");
    }

    @Test
    @DisplayName("The refractory period stops one piece being counted twice")
    void refractoryPreventsDoubleCount() {
      establishBaseline();

      // Load, briefly drop out as the piece tumbles, then load again — one physical piece.
      for (int i = 0; i < 10; i++) {
        monitor.update(IDLE_AMPS + 15, FULL_SPEED * 0.85, true);
        counter.update();
      }
      for (int i = 0; i < 2; i++) {
        monitor.update(IDLE_AMPS, FULL_SPEED, true);
        counter.update();
      }
      for (int i = 0; i < 10; i++) {
        monitor.update(IDLE_AMPS + 15, FULL_SPEED * 0.85, true);
        counter.update();
      }

      assertEquals(1, counter.getCount(),
          "Two current events inside the refractory window is one piece, not two");
    }

    @Test
    @DisplayName("Reset zeroes the count for a new match")
    void resetZeroesCount() {
      establishBaseline();
      onePieceThroughRollers();
      assertEquals(1, counter.getCount());

      counter.reset();
      assertEquals(0, counter.getCount());
    }
  }

  /**
   * The shooter's expected speed is its live setpoint, not a constant.
   *
   * <p>These cover the case that motivated the supplier overload: the same absolute RPM must read as
   * healthy at a low setpoint and as collapsed at a high one. With a fixed expected speed the
   * monitor can only be right at one setpoint out of five.
   */
  @Nested
  @DisplayName("A mechanism whose expected speed changes with its setpoint")
  class VariableSetpoint {

    private double setpoint = 4500;
    private MotorLoadMonitor flywheel;

    @BeforeEach
    void setUp() {
      flywheel = new MotorLoadMonitor("Test/Flywheel", 20.0, () -> setpoint, 0.70, 10);
    }

    /** Runs at the current setpoint long enough to learn the baseline. */
    private void establishBaselineAt(double rpm) {
      setpoint = rpm;
      for (int i = 0; i < 60; i++) {
        flywheel.update(IDLE_AMPS, rpm, true);
      }
    }

    @Test
    @DisplayName("2000 RPM is healthy at a 2100 setpoint")
    void healthyAtLowSetpoint() {
      establishBaselineAt(2100);

      for (int i = 0; i < 20; i++) {
        flywheel.update(IDLE_AMPS + 25, 2000, true);
      }

      assertEquals(LoadState.DOING_WORK, flywheel.getState(),
          "2000 of a 2100 setpoint is 95% — a piece going through, not a jam");
    }

    @Test
    @DisplayName("the same 2000 RPM is a jam at a 4500 setpoint")
    void jammedAtHighSetpoint() {
      establishBaselineAt(4500);

      for (int i = 0; i < 20; i++) {
        flywheel.update(IDLE_AMPS + 25, 2000, true);
      }

      assertEquals(LoadState.JAMMED, flywheel.getState(),
          "2000 of a 4500 setpoint is 44%, below the 70% threshold — something is rubbing");
    }

    @Test
    @DisplayName("changing the setpoint mid-run re-scales the judgement immediately")
    void followsTheSetpoint() {
      establishBaselineAt(4500);

      for (int i = 0; i < 20; i++) {
        flywheel.update(IDLE_AMPS + 25, 2000, true);
      }
      assertEquals(LoadState.JAMMED, flywheel.getState());

      // The operator drops to the HUB shot. Nothing about the mechanism changed, but 2000 RPM is
      // now nearly the whole setpoint, so the same readings are no longer a jam.
      setpoint = 2100;
      flywheel.update(IDLE_AMPS + 25, 2000, true);

      assertEquals(LoadState.DOING_WORK, flywheel.getState(),
          "the jam must clear as soon as the speed is healthy for the new setpoint");
    }
  }
}
