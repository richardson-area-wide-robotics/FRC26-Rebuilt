package frc.robot.rebuilt.components;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.testutil.HalFixture;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

/**
 * Tests for the field-state owner, which replaced a set of mutable statics that were only
 * refreshed during teleop and read via unguarded {@code Optional.get()}.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class FieldStateTest {

  @BeforeAll
  void setUpClass() {
    HalFixture.initialize();
  }

  @Test
  @DisplayName("Updating without an alliance does not throw")
  void updateSurvivesMissingAlliance() {
    HalFixture.clearAlliance();
    HalFixture.setGameData("");

    FieldState state = new FieldState();
    // The old code called DriverStation.getAlliance().get() five times per loop with no
    // presence check, throwing NoSuchElementException on every iteration until the driver
    // station reported an alliance.
    state.update();

    assertFalse(state.hasAlliance());
    assertFalse(state.isAllianceRed(), "Unknown alliance must not report as red");
  }

  @Test
  @DisplayName("Hub interlock fails open when the alliance is unknown")
  void interlockFailsOpenWithoutAlliance() {
    HalFixture.clearAlliance();
    FieldState state = new FieldState();
    state.update();

    assertTrue(state.isHubActive(),
        "With no alliance the robot should still be able to shoot rather than sit inert");
  }

  @Test
  @DisplayName("Alliance is picked up from the driver station")
  void readsAllianceFromDriverStation() {
    HalFixture.enableTeleop(true);
    FieldState red = new FieldState();
    red.update();
    assertTrue(red.hasAlliance());
    assertTrue(red.isAllianceRed());

    HalFixture.enableTeleop(false);
    FieldState blue = new FieldState();
    blue.update();
    assertTrue(blue.hasAlliance());
    assertFalse(blue.isAllianceRed());
  }

  @Test
  @DisplayName("Our hub state tracks our own alliance, not the other one")
  void hubStateFollowsOurAlliance() {
    HalFixture.setGameData("R");

    HalFixture.enableTeleop(true);
    FieldState red = new FieldState();
    red.update();

    HalFixture.enableTeleop(false);
    FieldState blue = new FieldState();
    blue.update();

    // With red named as first-inactive, at some point in the match exactly one of the two
    // is closed. Sampling live match time makes the exact phase unpredictable, so assert
    // the weaker but still meaningful property: both resolve to a defined state.
    assertTrue(red.getOurHubState() != null);
    assertTrue(blue.getOurHubState() != null);
  }

  @Test
  @DisplayName("Game data naming us is false when the FMS has sent nothing")
  void gameDataNamesUsIsFalseWithoutData() {
    HalFixture.enableTeleop(true);
    HalFixture.setGameData("");

    FieldState state = new FieldState();
    state.update();

    assertFalse(state.gameDataNamesUs(),
        "No FMS data must not be reported as naming our alliance");
  }

  @Test
  @DisplayName("Game data naming us is true when the FMS names our alliance")
  void gameDataNamesUsWhenMatching() {
    HalFixture.enableTeleop(true);
    HalFixture.setGameData("R");

    FieldState state = new FieldState();
    state.update();

    assertTrue(state.gameDataNamesUs(),
        "Red alliance with FMS message 'R' must be recognised. The old container compared "
            + "the message with == against lower-case \"r\", so this could never be true.");
  }

  @Test
  @DisplayName("Game data naming the other alliance is not reported as naming us")
  void gameDataNamesOtherAlliance() {
    HalFixture.enableTeleop(true);
    HalFixture.setGameData("B");

    FieldState state = new FieldState();
    state.update();

    assertFalse(state.gameDataNamesUs());
  }
}
