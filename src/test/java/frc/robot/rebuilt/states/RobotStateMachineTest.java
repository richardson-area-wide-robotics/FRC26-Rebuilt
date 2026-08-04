package frc.robot.rebuilt.states;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
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
    @DisplayName("Aims the shooter at the hub, which is not the same as aiming the chassis")
    void headingPointsShooterAtHub() {
      // Directly down-field of the blue hub, so the hub lies in the −x direction: bearing 180.
      Pose2d pose = new Pose2d(7.5, 4, new Rotation2d());
      StateOutput output = machine.update(
          pose, STILL, BLUE_HUB, FIELD_LENGTH, false, true, true);

      Rotation2d commanded = output.headingTarget().orElseThrow();

      // This test used to assert 180 — the bearing — which was asserting the bug. The shooter fires
      // 90 degrees off the chassis nose, so commanding 180 would have pointed the intake at the hub
      // and fired the shot across the field.
      assertNotEquals(180.0, commanded.getDegrees(), 1e-6,
          "commanding the bearing aims the intake, not the shooter");

      // What must hold: once the chassis is at the commanded heading, the shooter is on 180.
      Rotation2d shooterPointsAt = commanded.plus(Rotation2d.fromDegrees(
          frc.robot.rebuilt.RebuiltConstants.GeometryConstants.SHOOTER_YAW_OFFSET_DEGREES));

      assertEquals(180.0, shooterPointsAt.getDegrees(), 1e-6,
          "the shooter, not the nose, has to end up pointing down-field at the hub");
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
    @DisplayName("Bearing to hub is the plain direction from robot to hub")
    void bearingToHub() {
      Pose2d robot = new Pose2d(0, 0, new Rotation2d());
      Pose2d hub = new Pose2d(1, 1, new Rotation2d());
      assertEquals(45.0,
          ShooterRangeModel.bearingToHub(robot, hub).getDegrees(), 1e-6);
    }

    @Test
    @DisplayName("The aiming heading offsets the bearing by the shooter's mounting angle")
    void aimingHeadingAccountsForTheShooterOffset() {
      // The shooter fires 90 degrees off the chassis nose on this robot. So to put the shooter on a
      // hub that lies at a bearing of 45 degrees, the chassis has to sit at 45 - 90 = -45.
      Pose2d robot = new Pose2d(0, 0, new Rotation2d());
      Pose2d hub = new Pose2d(1, 1, new Rotation2d());

      double bearing = ShooterRangeModel.bearingToHub(robot, hub).getDegrees();
      double aim = ShooterRangeModel.headingToAimShooter(robot, hub).getDegrees();

      assertEquals(45.0, bearing, 1e-6);
      assertEquals(-45.0, aim, 1e-6);

      // The two must NOT be equal. Before this was fixed the state machine commanded the bearing
      // directly, which aimed the intake at the hub and fired the shot sideways off the field.
      assertNotEquals(bearing, aim,
          "aiming the chassis nose at the hub points the intake at it, not the shooter");
    }

    @Test
    @DisplayName("Once at the aiming heading, the shooter really does point at the hub")
    void shooterEndsUpPointingAtTheHub() {
      // The property that matters, checked at several bearings rather than trusting one subtraction.
      Pose2d robot = new Pose2d(2.0, 3.0, new Rotation2d());

      for (double[] hubXy : new double[][] {{5, 3}, {2, 8}, {-1, 3}, {2, -2}, {6, 7}}) {
        Pose2d hub = new Pose2d(hubXy[0], hubXy[1], new Rotation2d());

        Rotation2d chassis = ShooterRangeModel.headingToAimShooter(robot, hub);
        Rotation2d shooterPointsAt = chassis.plus(Rotation2d.fromDegrees(
            frc.robot.rebuilt.RebuiltConstants.GeometryConstants.SHOOTER_YAW_OFFSET_DEGREES));

        Rotation2d bearing = ShooterRangeModel.bearingToHub(robot, hub);

        assertEquals(0.0, shooterPointsAt.minus(bearing).getDegrees(), 1e-6,
            "shooter must end up on the hub bearing for hub at "
                + hubXy[0] + "," + hubXy[1]);
      }
    }

    @Test
    @DisplayName("Distance to hub is plain Euclidean distance")
    void distanceToHub() {
      assertEquals(5.0, ShooterRangeModel.distanceToHub(
          new Pose2d(0, 0, new Rotation2d()), new Pose2d(3, 4, new Rotation2d())), 1e-9);
    }
  }
}
