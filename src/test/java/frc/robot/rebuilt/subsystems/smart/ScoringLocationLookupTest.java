package frc.robot.rebuilt.subsystems.smart;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Tests for the nearest-scoring-location lookup.
 *
 * <p>Pure geometry — no HAL needed.
 */
class ScoringLocationLookupTest {

  @BeforeEach
  void setUp() {
    ScoringLocationLookup.buildScoringLocations();
    ScoringLocationLookup.clearAlliance();
  }

  @Test
  @DisplayName("Table reports itself built once populated")
  void tableIsBuilt() {
    assertTrue(ScoringLocationLookup.isBuilt());
  }

  @Test
  @DisplayName("Unknown alliance yields an empty pose rather than a wrong one")
  void unknownAllianceReturnsEmptyPose() {
    assertFalse(ScoringLocationLookup.hasAlliance());

    Pose2d result = ScoringLocationLookup.findClosest(new Pose2d(13, 4, new Rotation2d()));
    assertEquals(0.0, result.getX(), 1e-9);
    assertEquals(0.0, result.getY(), 1e-9);

    Pose2d hub = ScoringLocationLookup.findHub();
    assertEquals(0.0, hub.getX(), 1e-9);
    assertEquals(0.0, hub.getY(), 1e-9);
  }

  @Test
  @DisplayName("A null robot pose is handled rather than throwing")
  void nullPoseIsSafe() {
    ScoringLocationLookup.setRedAlliance(true);
    Pose2d result = ScoringLocationLookup.findClosest(null);
    assertEquals(0.0, result.getX(), 1e-9);
  }

  @Test
  @DisplayName("Red hub and blue hub are on opposite sides of the field")
  void hubsAreMirrored() {
    ScoringLocationLookup.setRedAlliance(true);
    Pose2d red = ScoringLocationLookup.findHub();

    ScoringLocationLookup.setRedAlliance(false);
    Pose2d blue = ScoringLocationLookup.findHub();

    assertNotEquals(red.getX(), blue.getX(), "Hubs must not share an x coordinate");
    assertTrue(red.getX() > blue.getX(), "Red hub sits further down field than blue");
  }

  @Test
  @DisplayName("Nearest location on red is the red-side pose")
  void findsNearestOnRed() {
    ScoringLocationLookup.setRedAlliance(true);

    // Sitting essentially on the red hub.
    Pose2d result = ScoringLocationLookup.findClosest(new Pose2d(13.0, 4.0, new Rotation2d()));

    assertEquals(13.0, result.getX(), 1e-6);
    assertEquals(4.0, result.getY(), 1e-6);
    assertEquals("hub", ScoringLocationLookup.findClosestName(
        new Pose2d(13.0, 4.0, new Rotation2d())));
  }

  @Test
  @DisplayName("Nearest location on blue is the blue-side pose")
  void findsNearestOnBlue() {
    ScoringLocationLookup.setRedAlliance(false);

    Pose2d result = ScoringLocationLookup.findClosest(new Pose2d(3.5, 4.0, new Rotation2d()));

    assertEquals(3.5, result.getX(), 1e-6);
    assertEquals(4.0, result.getY(), 1e-6);
    assertEquals("hub", ScoringLocationLookup.findClosestName(
        new Pose2d(3.5, 4.0, new Rotation2d())));
  }

  @Test
  @DisplayName("The same robot pose can resolve differently per alliance")
  void allianceChangesTheAnswer() {
    Pose2d probe = new Pose2d(8.0, 4.0, new Rotation2d());

    ScoringLocationLookup.setRedAlliance(true);
    Pose2d red = ScoringLocationLookup.findClosest(probe);

    ScoringLocationLookup.setRedAlliance(false);
    Pose2d blue = ScoringLocationLookup.findClosest(probe);

    assertNotEquals(red.getX(), blue.getX(),
        "Mid-field, the nearest red location and nearest blue location must differ");
  }

  @Test
  @DisplayName("Corner positions resolve to a corner")
  void findsCorner() {
    ScoringLocationLookup.setRedAlliance(true);
    String name = ScoringLocationLookup.findClosestName(new Pose2d(16, 7.5, new Rotation2d()));
    assertEquals("right_corner", name);
  }

  @Test
  @DisplayName("Every returned pose is one of the eight defined locations")
  void resultIsAlwaysAKnownLocation() {
    ScoringLocationLookup.setRedAlliance(true);

    // Sweep the field and confirm the lookup never invents a pose.
    for (double x = 0; x <= 16; x += 2) {
      for (double y = 0; y <= 8; y += 2) {
        String name = ScoringLocationLookup.findClosestName(new Pose2d(x, y, new Rotation2d()));
        assertFalse(name.isEmpty(),
            "Lookup must name a location for field position (" + x + ", " + y + ")");
      }
    }
  }
}
