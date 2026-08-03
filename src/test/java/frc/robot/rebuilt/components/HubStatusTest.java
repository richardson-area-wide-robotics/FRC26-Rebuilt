package frc.robot.rebuilt.components;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.rebuilt.components.HubStatus.AllianceGoalInactive;
import frc.robot.rebuilt.components.HubStatus.HubState;
import frc.robot.testutil.HalFixture;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

/**
 * Tests for the HUB shift model — the most game-specific logic in the codebase, and the part
 * a driver is most likely to blame the robot for when it disagrees with the field.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class HubStatusTest {

  @BeforeAll
  void setUpClass() {
    HalFixture.initialize();
  }

  @BeforeEach
  void setUp() {
    HalFixture.setGameData("R");
  }

  @Test
  @DisplayName("FMS message is parsed case-correctly for both alliances")
  void parsesAllianceFromGameData() {
    HalFixture.setGameData("R");
    assertEquals(AllianceGoalInactive.RED, HubStatus.getFirstInactiveAlliance());

    HalFixture.setGameData("B");
    assertEquals(AllianceGoalInactive.BLUE, HubStatus.getFirstInactiveAlliance());
  }

  @Test
  @DisplayName("Missing FMS data is distinguished from corrupt FMS data")
  void distinguishesMissingFromCorrupt() {
    HalFixture.setGameData("");
    assertEquals(AllianceGoalInactive.UNKNOWN_NO_DATA, HubStatus.getFirstInactiveAlliance());

    HalFixture.setGameData("X");
    assertEquals(AllianceGoalInactive.UNKNOWN_CORRUPT_DATA, HubStatus.getFirstInactiveAlliance());
  }

  @Test
  @DisplayName("Lower-case FMS data is treated as corrupt, not silently accepted")
  void lowerCaseIsCorrupt() {
    // This matters: the container used to compare the raw message against lower-case "r"
    // and "b", which never matched. Documenting the real casing prevents that reappearing.
    HalFixture.setGameData("r");
    assertEquals(AllianceGoalInactive.UNKNOWN_CORRUPT_DATA, HubStatus.getFirstInactiveAlliance());
  }

  @Test
  @DisplayName("Both hubs are active throughout autonomous")
  void bothHubsActiveInAuto() {
    // Auto occupies match time 130..150.
    for (double t : new double[] {131, 140, 150}) {
      assertEquals(HubState.ACTIVE, HubStatus.getHubStatus(DriverStation.Alliance.Red, t));
      assertEquals(HubState.ACTIVE, HubStatus.getHubStatus(DriverStation.Alliance.Blue, t));
    }
  }

  @Test
  @DisplayName("Both hubs are active during endgame")
  void bothHubsActiveInEndgame() {
    for (double t : new double[] {0, 15, 29}) {
      assertEquals(HubState.ACTIVE, HubStatus.getHubStatus(DriverStation.Alliance.Red, t));
      assertEquals(HubState.ACTIVE, HubStatus.getHubStatus(DriverStation.Alliance.Blue, t));
    }
  }

  @Test
  @DisplayName("Exactly one alliance is inactive during a numbered shift")
  void shiftsAlternateBetweenAlliances() {
    HalFixture.setGameData("R");

    // Mid-shift sample points, away from the blink windows at 125/105/80/55/30.
    double[] midShift = {115, 95, 70, 45};

    for (double t : midShift) {
      HubState red = HubStatus.getHubStatus(DriverStation.Alliance.Red, t);
      HubState blue = HubStatus.getHubStatus(DriverStation.Alliance.Blue, t);
      assertNotEquals(red, blue,
          "At match time " + t + " the two alliances must not share a hub state");
    }
  }

  @Test
  @DisplayName("Red goes inactive first when the FMS names red")
  void redInactiveFirstWhenNamed() {
    HalFixture.setGameData("R");
    // Shift 1 is 105 < t <= 125; sample at 115.
    assertEquals(HubState.INACTIVE, HubStatus.getHubStatus(DriverStation.Alliance.Red, 115));
    assertEquals(HubState.ACTIVE, HubStatus.getHubStatus(DriverStation.Alliance.Blue, 115));
  }

  @Test
  @DisplayName("Blue goes inactive first when the FMS names blue")
  void blueInactiveFirstWhenNamed() {
    HalFixture.setGameData("B");
    assertEquals(HubState.INACTIVE, HubStatus.getHubStatus(DriverStation.Alliance.Blue, 115));
    assertEquals(HubState.ACTIVE, HubStatus.getHubStatus(DriverStation.Alliance.Red, 115));
  }

  @Test
  @DisplayName("State flips between consecutive shifts")
  void stateFlipsBetweenShifts() {
    HalFixture.setGameData("R");
    HubState shift1 = HubStatus.getHubStatus(DriverStation.Alliance.Red, 115);
    HubState shift2 = HubStatus.getHubStatus(DriverStation.Alliance.Red, 95);
    assertNotEquals(shift1, shift2, "Red's hub must alternate from one shift to the next");
  }

  @Test
  @DisplayName("Transition shift before the first numbered shift is active")
  void transitionShiftIsActive() {
    // 125 < t <= 130 is the transition window.
    assertEquals(HubState.ACTIVE, HubStatus.getHubStatus(DriverStation.Alliance.Red, 128));
  }

  @Test
  @DisplayName("Blink windows straddle every shift boundary")
  void blinkNearBoundaries() {
    HalFixture.setGameData("R");
    // Within 1s either side of a boundary the state is derived from a 4 Hz blink, so it is
    // one of ACTIVE/INACTIVE and changes over time rather than being fixed by shift parity.
    for (double boundary : new double[] {125, 105, 80, 55}) {
      HubState atBoundary = HubStatus.getHubStatus(DriverStation.Alliance.Red, boundary);
      assertTrue(atBoundary == HubState.ACTIVE || atBoundary == HubState.INACTIVE,
          "Blink window must resolve to a concrete on/off state");
    }
  }

  @Test
  @DisplayName("Both-hub helper agrees with the single-hub query")
  void bothStatusesMatchIndividualQueries() {
    HalFixture.setGameData("R");
    double t = 115;

    HubState[] both = HubStatus.getBothHubStatuses(t);
    assertEquals(HubStatus.getHubStatus(DriverStation.Alliance.Red, t), both[0],
        "Index 0 must be red");
    assertEquals(HubStatus.getHubStatus(DriverStation.Alliance.Blue, t), both[1],
        "Index 1 must be blue");
  }

  @Test
  @DisplayName("Missing FMS data does not throw at any point in the match")
  void missingDataIsSafe() {
    HalFixture.setGameData("");
    for (double t = 150; t >= 0; t -= 5) {
      HubState red = HubStatus.getHubStatus(DriverStation.Alliance.Red, t);
      HubState blue = HubStatus.getHubStatus(DriverStation.Alliance.Blue, t);
      assertTrue(red != null && blue != null, "Hub state must be defined at match time " + t);
    }
  }
}
