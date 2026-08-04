package frc.robot.rebuilt.states;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.rebuilt.states.RobotStateMachine.State;
import frc.robot.rebuilt.states.RobotStateMachine.StateOutput;
import frc.robot.rebuilt.subsystems.smart.ScoringLocationLookup;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Tests for the localisation-driven behaviour selection.
 *
 * <p>These states move the robot without being asked, so the conditions under which they engage
 * — and just as importantly, refuse to engage — are worth pinning down precisely.
 */
class RobotStateMachineTest {

  /** 2026 field length, roughly. Exact value does not matter for midfield logic. */
  private static final double FIELD_LENGTH = 17.5;

  /** Blue hub, from ScoringLocationLookup. */
  private static final Pose2d BLUE_HUB = new Pose2d(4.5, 4, new Rotation2d());

  /** Red hub, from ScoringLocationLookup. */
  private static final Pose2d RED_HUB = new Pose2d(11, 4, new Rotation2d());

  private static final Pose2d NO_HUB = new Pose2d();

  private RobotStateMachine machine;

  @BeforeEach
  void setUp() {
    machine = new RobotStateMachine();
  }

  /** Stationary, so bump logic stays out of the way unless a test wants it. */
  private static final Translation2d STILL = new Translation2d();

  @Nested
  @DisplayName("Aim at hub")
  class AimAtHub {

    @Test
    @DisplayName("Engages on our own side with the hub open")
    void engagesOnOwnSideWithHubOpen() {
      // Blue, 3 m from the blue hub, well inside our own half.
      Pose2d pose = new Pose2d(7.5, 4, new Rotation2d());
      StateOutput output = machine.update(
          pose, STILL, BLUE_HUB, FIELD_LENGTH, false, true, true);

      assertEquals(State.AIM_AT_HUB, output.state());
      assertTrue(output.hasHeadingTarget());
      assertTrue(output.hasShooterTarget());
    }

    @Test
    @DisplayName("Points back down the field at the hub, not away from it")
    void headingPointsAtHub() {
      // Directly down-field of the blue hub, so the hub is in the −x direction: 180 degrees.
      Pose2d pose = new Pose2d(7.5, 4, new Rotation2d());
      StateOutput output = machine.update(
          pose, STILL, BLUE_HUB, FIELD_LENGTH, false, true, true);

      assertEquals(180.0, output.headingTarget().orElseThrow().getDegrees(), 1e-6);
    }

    @Test
    @DisplayName("Speed increases with distance from the hub")
    void speedIncreasesWithRange() {
      double near = machine.update(new Pose2d(7.0, 4, new Rotation2d()),
          STILL, BLUE_HUB, FIELD_LENGTH, false, true, true)
          .shooterRpm().orElseThrow();

      machine.reset();
      double far = machine.update(new Pose2d(8.5, 4, new Rotation2d()),
          STILL, BLUE_HUB, FIELD_LENGTH, false, true, true)
          .shooterRpm().orElseThrow();

      assertTrue(far > near,
          "A longer shot needs more speed; near=" + near + " far=" + far);
    }

    @Test
    @DisplayName("Does not engage while our hub is closed")
    void staysManualWhenHubClosed() {
      StateOutput output = machine.update(new Pose2d(7.5, 4, new Rotation2d()),
          STILL, BLUE_HUB, FIELD_LENGTH, false, false, true);

      assertEquals(State.MANUAL, output.state(),
          "Spinning up into a closed goal is exactly what the interlock exists to prevent");
    }

    @Test
    @DisplayName("Does not engage on the opponent's half")
    void staysManualOnOpponentSide() {
      // Blue robot past midfield.
      StateOutput output = machine.update(new Pose2d(12.0, 4, new Rotation2d()),
          STILL, BLUE_HUB, FIELD_LENGTH, false, true, true);

      assertEquals(State.MANUAL, output.state());
    }

    @Test
    @DisplayName("Own side is mirrored for red")
    void ownSideIsMirroredForRed() {
      // x = 12 is the opponent half for blue but our own half for red.
      StateOutput red = machine.update(new Pose2d(12.0, 4, new Rotation2d()),
          STILL, RED_HUB, FIELD_LENGTH, true, true, true);
      assertEquals(State.AIM_AT_HUB, red.state());
    }

    @Test
    @DisplayName("Does nothing when the hub pose is unknown")
    void staysManualWithoutHubPose() {
      StateOutput output = machine.update(new Pose2d(7.5, 4, new Rotation2d()),
          STILL, NO_HUB, FIELD_LENGTH, false, true, true);
      assertEquals(State.MANUAL, output.state());
    }
  }

  @Nested
  @DisplayName("Bump crossing")
  class BumpCrossing {

    /** Moving down-field, towards the red end. */
    private static final Translation2d MOVING_FORWARD = new Translation2d(1.0, 0);

    @Test
    @DisplayName("Engages when approaching the bump")
    void engagesOnApproach() {
      // Just before the near edge, moving towards it.
      Pose2d pose = new Pose2d(FieldRegions.BUMP_NEAR_EDGE_METERS - 0.5, 4, new Rotation2d());
      StateOutput output = machine.update(
          pose, MOVING_FORWARD, BLUE_HUB, FIELD_LENGTH, false, true, true);

      assertEquals(State.BUMP_REVERSE, output.state());
      assertTrue(output.hasHeadingTarget());
    }

    @Test
    @DisplayName("Targets a heading opposite the direction of travel, so the back leads")
    void facesBackwards() {
      Pose2d pose = new Pose2d(FieldRegions.BUMP_NEAR_EDGE_METERS + 0.5, 4, new Rotation2d());
      StateOutput output = machine.update(
          pose, MOVING_FORWARD, BLUE_HUB, FIELD_LENGTH, false, true, true);

      // Travelling towards +x, so the robot should face −x: 180 degrees.
      assertEquals(180.0,
          Math.abs(output.headingTarget().orElseThrow().getDegrees()), 1e-6);
    }

    @Test
    @DisplayName("Does not spin around when sitting past the bump driving away")
    void ignoresBumpWhenLeaving() {
      // Past the far edge and still moving away from it.
      Pose2d pose = new Pose2d(FieldRegions.BUMP_FAR_EDGE_METERS + 0.5, 4, new Rotation2d());
      StateOutput output = machine.update(
          pose, MOVING_FORWARD, RED_HUB, FIELD_LENGTH, true, true, true);

      assertFalse(output.state() == State.BUMP_REVERSE,
          "Driving away from the bump must not trigger a turn");
    }

    @Test
    @DisplayName("Stationary near the bump does not trigger a turn")
    void stationaryDoesNotTrigger() {
      Pose2d pose = new Pose2d(FieldRegions.BUMP_NEAR_EDGE_METERS - 0.5, 4, new Rotation2d());
      StateOutput output = machine.update(
          pose, STILL, BLUE_HUB, FIELD_LENGTH, false, true, true);

      assertFalse(output.state() == State.BUMP_REVERSE,
          "A robot that is not moving has not committed to crossing anything");
    }

    @Test
    @DisplayName("Bump crossing outranks aiming")
    void bumpBeatsAiming() {
      // On the bump, blue's own half, hub open — both states would otherwise apply.
      Pose2d onBump = new Pose2d(
          (FieldRegions.BUMP_NEAR_EDGE_METERS + FieldRegions.BUMP_FAR_EDGE_METERS) / 2,
          4, new Rotation2d());

      StateOutput output = machine.update(
          onBump, MOVING_FORWARD, BLUE_HUB, FIELD_LENGTH, false, true, true);

      assertEquals(State.BUMP_REVERSE, output.state(),
          "Crossing an obstacle correctly matters more than optimising a shot");
      assertFalse(output.hasShooterTarget(),
          "The bump state should not be commanding flywheel speed");
    }

    @Test
    @DisplayName("On the bump counts as engaged regardless of approach direction")
    void onBumpAlwaysEngages() {
      Pose2d onBump = new Pose2d(FieldRegions.BUMP_NEAR_EDGE_METERS + 0.1, 4, new Rotation2d());
      // Moving backwards, towards the blue end.
      StateOutput output = machine.update(
          onBump, new Translation2d(-1.0, 0), BLUE_HUB, FIELD_LENGTH, false, true, true);

      assertEquals(State.BUMP_REVERSE, output.state());
    }
  }

  @Nested
  @DisplayName("Safety")
  class Safety {

    @Test
    @DisplayName("Everything falls back to manual when the pose is not trustworthy")
    void untrustworthyPoseDisablesAssists() {
      // Conditions that would otherwise trigger aiming.
      StateOutput output = machine.update(new Pose2d(7.5, 4, new Rotation2d()),
          STILL, BLUE_HUB, FIELD_LENGTH, false, true, false);

      assertEquals(State.MANUAL, output.state());
      assertFalse(output.hasHeadingTarget(),
          "A heading assist fighting the driver from a wrong pose is the worst failure mode");
      assertFalse(output.hasShooterTarget());
    }

    @Test
    @DisplayName("Untrustworthy pose also disables the bump assist")
    void untrustworthyPoseDisablesBump() {
      Pose2d onBump = new Pose2d(FieldRegions.BUMP_NEAR_EDGE_METERS + 0.1, 4, new Rotation2d());
      StateOutput output = machine.update(onBump, new Translation2d(1, 0),
          BLUE_HUB, FIELD_LENGTH, false, true, false);

      assertEquals(State.MANUAL, output.state());
    }

    @Test
    @DisplayName("Manual state never requests heading or shooter control")
    void manualRequestsNothing() {
      StateOutput output = machine.update(new Pose2d(12.0, 4, new Rotation2d()),
          STILL, BLUE_HUB, FIELD_LENGTH, false, false, true);

      assertEquals(State.MANUAL, output.state());
      assertFalse(output.hasHeadingTarget());
      assertFalse(output.hasShooterTarget());
    }

    @Test
    @DisplayName("Every state reports a human-readable reason")
    void statesExplainThemselves() {
      StateOutput manual = machine.update(new Pose2d(12.0, 4, new Rotation2d()),
          STILL, BLUE_HUB, FIELD_LENGTH, false, false, true);
      assertFalse(manual.reason().isBlank());

      machine.reset();
      StateOutput aim = machine.update(new Pose2d(7.5, 4, new Rotation2d()),
          STILL, BLUE_HUB, FIELD_LENGTH, false, true, true);
      assertTrue(aim.reason().contains("m"), "Aim reason should quote the range: " + aim.reason());
    }
  }

  @Nested
  @DisplayName("Range model")
  class RangeModel {

    @Test
    @DisplayName("Preset distances return their tuned speeds")
    void presetsMatch() {
      assertEquals(2100, ShooterRangeModel.rpmForDistance(2.00), 1.0);
      assertEquals(2900, ShooterRangeModel.rpmForDistance(3.40), 1.0);
      assertEquals(3250, ShooterRangeModel.rpmForDistance(4.13), 1.0);
      assertEquals(4500, ShooterRangeModel.rpmForDistance(6.10), 1.0);
    }

    @Test
    @DisplayName("Table distances actually match the scoring-location geometry they claim to")
    void tableDistancesMatchFieldGeometry() {
      // The test above is circular on its own: 4.13 is one of the table's own keys, so it
      // proves interpolation works, not that the distance is right. This couples the table to
      // the geometry it was derived from, so moving a scoring-location pose fails a test rather
      // than silently invalidating the whole speed curve.
      ScoringLocationLookup.buildScoringLocations();
      ScoringLocationLookup.setRedAlliance(true);

      Pose2d hub = ScoringLocationLookup.findHub();

      // Probing at each location's own coordinates returns that location, so these read the
      // real table without needing new accessors.
      record Expected(String name, Pose2d probe, double tableDistance) { }
      var cases = new Expected[] {
          new Expected("hub", new Pose2d(13.0, 4.0, new Rotation2d()), 2.00),
          new Expected("right_trench", new Pose2d(13.2, 7.5, new Rotation2d()), 4.13),
          new Expected("right_corner", new Pose2d(16.0, 7.5, new Rotation2d()), 6.10)
      };

      for (Expected expected : cases) {
        Pose2d location = ScoringLocationLookup.findClosest(expected.probe());
        double actual = ShooterRangeModel.distanceToHub(location, hub);

        assertEquals(expected.tableDistance(), actual, 0.02,
            "ShooterRangeModel's table says " + expected.tableDistance() + " m for "
                + expected.name() + ", but its actual distance to the hub is " + actual
                + " m. Either the table or the scoring-location pose has moved, and the speed "
                + "curve is now wrong for that range.");
      }

      ScoringLocationLookup.clearAlliance();
    }

    @Test
    @DisplayName("Speed is monotonic across the whole calibrated range")
    void monotonic() {
      double previous = 0;
      for (double d = ShooterRangeModel.MIN_RANGE_METERS;
          d <= ShooterRangeModel.MAX_RANGE_METERS; d += 0.1) {
        double rpm = ShooterRangeModel.rpmForDistance(d);
        assertTrue(rpm >= previous,
            "Speed must not fall as range grows, at " + d + " m");
        previous = rpm;
      }
    }

    @Test
    @DisplayName("Beyond the calibrated range the speed clamps rather than extrapolating")
    void clampsOutsideRange() {
      assertEquals(ShooterRangeModel.rpmForDistance(ShooterRangeModel.MAX_RANGE_METERS),
          ShooterRangeModel.rpmForDistance(20.0), 1e-9,
          "A shot from further than anyone has tried should not be guessed at");
      assertEquals(ShooterRangeModel.rpmForDistance(ShooterRangeModel.MIN_RANGE_METERS),
          ShooterRangeModel.rpmForDistance(0.1), 1e-9);

      assertFalse(ShooterRangeModel.isInRange(20.0));
      assertTrue(ShooterRangeModel.isInRange(3.0));
    }

    @Test
    @DisplayName("Heading to hub is the bearing from robot to hub")
    void headingToHub() {
      Pose2d robot = new Pose2d(0, 0, new Rotation2d());
      Pose2d hub = new Pose2d(1, 1, new Rotation2d());
      assertEquals(45.0,
          ShooterRangeModel.headingToHub(robot, hub).getDegrees(), 1e-6);
    }

    @Test
    @DisplayName("Distance to hub is plain Euclidean distance")
    void distanceToHub() {
      assertEquals(5.0, ShooterRangeModel.distanceToHub(
          new Pose2d(0, 0, new Rotation2d()), new Pose2d(3, 4, new Rotation2d())), 1e-9);
    }
  }
}
