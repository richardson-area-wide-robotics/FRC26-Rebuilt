package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.common.components.diagnostics.CalibrationManeuvers.Leg;
import frc.robot.common.components.diagnostics.CalibrationManeuvers.Maneuver;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Tests for the manoeuvre catalogue and the geometry that scores it.
 *
 * <p>The expected-pose maths is the reference every live run is judged against, so an error
 * here would silently mis-score every manoeuvre on the field.
 */
class CalibrationManeuversTest {

  private static final double TEN_FT = CalibrationManeuvers.TEN_FEET_METERS;
  private static final double FIVE_FT = CalibrationManeuvers.FIVE_FEET_METERS;

  @Nested
  @DisplayName("Expected pose geometry")
  class Geometry {

    @Test
    @DisplayName("Ten feet is 3.048 m and five feet is 1.524 m")
    void distancesAreCorrect() {
      assertEquals(3.048, TEN_FT, 1e-9);
      assertEquals(1.524, FIVE_FT, 1e-9);
    }

    @Test
    @DisplayName("A single forward leg translates along the starting heading")
    void singleForwardLeg() {
      Pose2d end = CalibrationManeuvers.expectedPose(
          new Pose2d(), List.of(Leg.drive(TEN_FT)));
      assertEquals(TEN_FT, end.getX(), 1e-9);
      assertEquals(0.0, end.getY(), 1e-9);
      assertEquals(0.0, end.getRotation().getDegrees(), 1e-9);
    }

    @Test
    @DisplayName("A reverse leg moves backwards without turning around")
    void reverseLeg() {
      Pose2d end = CalibrationManeuvers.expectedPose(
          new Pose2d(), List.of(Leg.drive(-TEN_FT)));
      assertEquals(-TEN_FT, end.getX(), 1e-9);
      assertEquals(0.0, end.getRotation().getDegrees(), 1e-9,
          "Driving in reverse must not change heading");
    }

    @Test
    @DisplayName("Driving after a left turn moves along the new heading")
    void driveAfterTurn() {
      // Forward 10 ft, turn left 90, forward 5 ft: should end at (10ft, 5ft) facing 90.
      Pose2d end = CalibrationManeuvers.expectedPose(new Pose2d(), List.of(
          Leg.drive(TEN_FT), Leg.turn(90), Leg.drive(FIVE_FT)));

      assertEquals(TEN_FT, end.getX(), 1e-9);
      assertEquals(FIVE_FT, end.getY(), 1e-9);
      assertEquals(90.0, end.getRotation().getDegrees(), 1e-9);
    }

    @Test
    @DisplayName("A right turn goes the other way")
    void rightTurnIsNegative() {
      Pose2d end = CalibrationManeuvers.expectedPose(new Pose2d(), List.of(
          Leg.drive(TEN_FT), Leg.turn(-90), Leg.drive(FIVE_FT)));

      assertEquals(TEN_FT, end.getX(), 1e-9);
      assertEquals(-FIVE_FT, end.getY(), 1e-9);
      assertEquals(-90.0, end.getRotation().getDegrees(), 1e-9);
    }

    @Test
    @DisplayName("A 270 degree left turn ends up facing the same way as 90 right")
    void twoSeventyEqualsNinetyOther() {
      Pose2d left270 = CalibrationManeuvers.expectedPose(
          new Pose2d(), List.of(Leg.turn(270)));
      Pose2d right90 = CalibrationManeuvers.expectedPose(
          new Pose2d(), List.of(Leg.turn(-90)));

      assertEquals(right90.getRotation().getDegrees(), left270.getRotation().getDegrees(), 1e-9,
          "270 left and 90 right are the same final heading — which is exactly why the turn "
              + "command must accumulate rotation rather than servo to an absolute heading");
    }

    @Test
    @DisplayName("A closed square returns exactly to its start")
    void squareCloses() {
      Pose2d end = CalibrationManeuvers.expectedPose(new Pose2d(), List.of(
          Leg.drive(FIVE_FT), Leg.turn(90),
          Leg.drive(FIVE_FT), Leg.turn(90),
          Leg.drive(FIVE_FT), Leg.turn(90),
          Leg.drive(FIVE_FT), Leg.turn(90)));

      assertEquals(0.0, end.getX(), 1e-9);
      assertEquals(0.0, end.getY(), 1e-9);
      assertEquals(0.0, end.getRotation().getDegrees(), 1e-9);
    }

    @Test
    @DisplayName("An equilateral triangle closes too")
    void triangleCloses() {
      Pose2d end = CalibrationManeuvers.expectedPose(new Pose2d(), List.of(
          Leg.drive(FIVE_FT), Leg.turn(120),
          Leg.drive(FIVE_FT), Leg.turn(120),
          Leg.drive(FIVE_FT), Leg.turn(120)));

      assertEquals(0.0, end.getX(), 1e-6);
      assertEquals(0.0, end.getY(), 1e-6);
    }

    @Test
    @DisplayName("Geometry composes from a non-zero starting pose")
    void respectsStartingPose() {
      Pose2d start = new Pose2d(4.0, 2.0, Rotation2d.fromDegrees(90));
      Pose2d end = CalibrationManeuvers.expectedPose(start, List.of(Leg.drive(1.0)));

      // Facing +y, so driving forward increases y.
      assertEquals(4.0, end.getX(), 1e-9);
      assertEquals(3.0, end.getY(), 1e-9);
    }
  }

  @Nested
  @DisplayName("Retracing")
  class Retrace {

    @Test
    @DisplayName("Retracing an out-and-back returns to the start")
    void retraceCloses() {
      List<Leg> out = List.of(Leg.drive(TEN_FT), Leg.turn(90), Leg.drive(FIVE_FT));
      List<Leg> back = CalibrationManeuvers.retraceLegs(out);

      List<Leg> both = new java.util.ArrayList<>(out);
      both.addAll(back);

      Pose2d end = CalibrationManeuvers.expectedPose(new Pose2d(), both);
      assertEquals(0.0, end.getX(), 1e-9);
      assertEquals(0.0, end.getY(), 1e-9);
      assertEquals(0.0, end.getRotation().getDegrees(), 1e-9,
          "Retracing must restore the original heading as well as the original position");
    }

    @Test
    @DisplayName("Retracing reverses order and sign")
    void retraceReversesOrderAndSign() {
      List<Leg> out = List.of(Leg.drive(2.0), Leg.turn(45));
      List<Leg> back = CalibrationManeuvers.retraceLegs(out);

      assertEquals(2, back.size());
      assertEquals(-45.0, back.get(0).value(), 1e-9, "Last leg out is undone first");
      assertEquals(-2.0, back.get(1).value(), 1e-9);
    }
  }

  @Nested
  @DisplayName("Catalogue")
  class Catalogue {

    @Test
    @DisplayName("Permutations cover all sixteen combinations")
    void sixteenPermutations() {
      List<Maneuver> permutations = CalibrationManeuvers.permutations();
      // 2 distances x 2 turn magnitudes x 2 directions x 2 final directions.
      assertEquals(16, permutations.size());
    }

    @Test
    @DisplayName("Every manoeuvre name is unique, since names are log keys")
    void namesAreUnique() {
      List<Maneuver> all = CalibrationManeuvers.all();
      Set<String> names = new HashSet<>();
      for (Maneuver maneuver : all) {
        assertTrue(names.add(maneuver.name()),
            "Duplicate manoeuvre name would collide in the log: " + maneuver.name());
      }
    }

    @Test
    @DisplayName("Permutations include both distances, both turns and both directions")
    void permutationsAreComplete() {
      List<Maneuver> permutations = CalibrationManeuvers.permutations();

      long tenFoot = permutations.stream().filter(m -> m.name().contains("10ft_")).count();
      long fiveFoot = permutations.stream().filter(m -> m.name().contains("5ft_")).count();
      assertEquals(8, tenFoot);
      assertEquals(8, fiveFoot);

      long left = permutations.stream().filter(m -> m.name().contains("_L")).count();
      long right = permutations.stream().filter(m -> m.name().contains("_R")).count();
      assertEquals(8, left);
      assertEquals(8, right);

      long ninety = permutations.stream().filter(m -> m.name().contains("90_")).count();
      long twoSeventy = permutations.stream().filter(m -> m.name().contains("270_")).count();
      assertEquals(8, ninety);
      assertEquals(8, twoSeventy);

      long reverse = permutations.stream().filter(m -> m.name().endsWith("rev10ft")).count();
      assertEquals(8, reverse);
    }

    @Test
    @DisplayName("Permutations do not claim to return to the start")
    void permutationsAreOpenEnded() {
      for (Maneuver maneuver : CalibrationManeuvers.permutations()) {
        assertFalse(maneuver.returnsToStart(),
            maneuver.name() + " is drive-turn-drive, so it does not close a loop");
      }
    }

    @Test
    @DisplayName("Every loop manoeuvre really does close, geometrically")
    void loopsActuallyClose() {
      List<Maneuver> loops = new java.util.ArrayList<>();
      loops.addAll(CalibrationManeuvers.outAndBackSamePath());
      loops.addAll(CalibrationManeuvers.outAndBackDifferentPath());

      for (Maneuver maneuver : loops) {
        assertTrue(maneuver.returnsToStart(), maneuver.name() + " should be flagged as a loop");

        Pose2d end = CalibrationManeuvers.expectedPose(new Pose2d(), maneuver.legs());
        double closure = Math.hypot(end.getX(), end.getY());
        assertEquals(0.0, closure, 1e-6,
            maneuver.name() + " is flagged as returning to start but the geometry closes "
                + closure + " m away — closure error would be measured against the wrong "
                + "reference");
      }
    }

    @Test
    @DisplayName("Distance totals are sane and usable for session planning")
    void distanceTotals() {
      List<Maneuver> all = CalibrationManeuvers.all();
      double total = CalibrationManeuvers.totalDistanceMeters(all);

      assertTrue(total > 50, "The full catalogue should be a substantial session, got " + total);
      assertTrue(total < 500, "But not so long it cannot be run in one sitting, got " + total);

      for (Maneuver maneuver : all) {
        assertTrue(maneuver.totalDistanceMeters() > 0,
            maneuver.name() + " drives nowhere");
      }
    }

    @Test
    @DisplayName("Same-path and different-path families are both populated")
    void bothReturnFamiliesExist() {
      assertTrue(CalibrationManeuvers.outAndBackSamePath().size() >= 3);
      assertTrue(CalibrationManeuvers.outAndBackDifferentPath().size() >= 3);
    }

    @Test
    @DisplayName("Square manoeuvres exist in both rotational directions")
    void squaresBothWays() {
      List<Maneuver> different = CalibrationManeuvers.outAndBackDifferentPath();
      boolean left = different.stream().anyMatch(m ->
          m.legs().stream().anyMatch(l -> l.value() == 90));
      boolean right = different.stream().anyMatch(m ->
          m.legs().stream().anyMatch(l -> l.value() == -90));

      assertTrue(left && right,
          "Turn error is often direction-dependent, so both directions must be covered");
    }
  }
}
