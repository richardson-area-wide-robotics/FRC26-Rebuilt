package frc.robot.rebuilt.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.rebuilt.RebuiltConstants.FeederConstants;
import frc.robot.testutil.HalFixture;
import frc.robot.testutil.SharedSubsystems;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

/**
 * Tests for the feeder and spindexer, focused on the distinction between holding and
 * stopping — a distinction the original naming hid.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class FeederTest {

  private Feeder feeder;

  @BeforeAll
  void setUpClass() {
    HalFixture.initialize();
    feeder = SharedSubsystems.feeder();
  }

  @BeforeEach
  void setUp() {
    HalFixture.enableTeleop(true);
    feeder.stopLoad();
    feeder.stopCycle();
  }

  @Test
  @DisplayName("Loading drives the feeder forward")
  void loadDrivesFeeder() {
    feeder.load();
    assertEquals(FeederConstants.FEEDER_SPEED, feeder.getFeederDemand(), 1e-9);
    assertTrue(feeder.isLoading());
  }

  @Test
  @DisplayName("Cycling drives the spindexer forward")
  void cycleDrivesSpindexer() {
    feeder.cycle();
    assertEquals(FeederConstants.SPINDEXER_SPEED, feeder.getSpindexerDemand(), 1e-9);
    assertTrue(feeder.isCycling());
  }

  @Test
  @DisplayName("Reversing drives both backwards")
  void reverseDrivesBackwards() {
    feeder.reverseLoad();
    feeder.reverseCycle();

    assertEquals(-FeederConstants.FEEDER_SPEED, feeder.getFeederDemand(), 1e-9);
    assertEquals(-FeederConstants.SPINDEXER_SPEED, feeder.getSpindexerDemand(), 1e-9);
    assertFalse(feeder.isLoading(), "Reversing is not loading");
    assertFalse(feeder.isCycling(), "Reversing is not cycling");
  }

  @Test
  @DisplayName("Stopping the feeder really stops it")
  void stopLoadIsZero() {
    feeder.load();
    feeder.stopLoad();
    assertEquals(0.0, feeder.getFeederDemand(), 1e-9);
    assertFalse(feeder.isLoading());
  }

  @Test
  @DisplayName("Stopping the spindexer really stops it")
  void stopCycleIsZero() {
    feeder.cycle();
    feeder.stopCycle();
    assertEquals(0.0, feeder.getSpindexerDemand(), 1e-9,
        "stopCycle() must actually stop. The original method of this name set 0.1.");
    assertFalse(feeder.isCycling());
  }

  @Test
  @DisplayName("Holding the spindexer applies the retention crawl, not a stop")
  void holdCycleAppliesCrawl() {
    feeder.holdCycle();
    assertEquals(FeederConstants.SPINDEXER_HOLD_SPEED, feeder.getSpindexerDemand(), 1e-9);
    assertFalse(feeder.isCycling(),
        "A retention crawl is not indexing, so isCycling() must be false");
    assertTrue(feeder.getSpindexerDemand() > 0,
        "The crawl is deliberately non-zero — it keeps pieces from wedging");
  }

  @Test
  @DisplayName("The hold crawl is much slower than indexing")
  void holdIsSlowerThanCycle() {
    assertTrue(FeederConstants.SPINDEXER_HOLD_SPEED < FeederConstants.SPINDEXER_SPEED / 2,
        "A hold that is close to full speed is not a hold");
  }

  @Test
  @DisplayName("Named auto commands drive both mechanisms together")
  void namedCommandsAffectBoth() {
    feeder.loadAndCycleCommand().initialize();
    assertTrue(feeder.isLoading());
    assertTrue(feeder.isCycling());

    feeder.stopLoadAndCycleCommand().initialize();
    assertFalse(feeder.isLoading());
    assertEquals(FeederConstants.SPINDEXER_HOLD_SPEED, feeder.getSpindexerDemand(), 1e-9,
        "Disabling load should leave the spindexer holding, not stopped");
  }
}
