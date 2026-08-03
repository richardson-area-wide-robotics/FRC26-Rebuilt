package frc.robot.common.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Tests for the vision trust model and the field layout.
 *
 * <p>The trust model decides how much each AprilTag measurement moves the pose estimate. Get
 * it wrong in one direction and vision does nothing; wrong in the other and the robot jitters
 * across the field.
 */
class VisionTrustModelTest {

  @Test
  @DisplayName("Trust falls off with the square of tag distance")
  void trustFallsOffWithDistance() {
    double near = xyStdDev(1, 1.0);
    double mid = xyStdDev(1, 2.0);
    double far = xyStdDev(1, 4.0);

    assertTrue(mid > near, "A more distant tag must be trusted less");
    assertTrue(far > mid);

    // Doubling distance should roughly quadruple the standard deviation.
    assertEquals(4.0, mid / near, 0.01);
    assertEquals(4.0, far / mid, 0.01);
  }

  @Test
  @DisplayName("Multiple tags are trusted more than one")
  void multiTagIsTrustedMore() {
    double single = xyStdDev(1, 3.0);
    double dual = xyStdDev(2, 3.0);
    double triple = xyStdDev(3, 3.0);

    assertTrue(dual < single,
        "A multi-tag solve is geometrically constrained, so it deserves more trust");
    assertTrue(triple < dual);
  }

  @Test
  @DisplayName("Standard deviations never collapse to zero or go negative")
  void trustStaysPositive() {
    for (int tags = 1; tags <= 8; tags++) {
      for (double distance = 0.1; distance <= 8.0; distance += 0.5) {
        Matrix<N3, N1> stdDevs = VisionSubsystem.computeStdDevs(tags, distance);
        for (int row = 0; row < 3; row++) {
          double value = stdDevs.get(row, 0);
          assertTrue(value > 0,
              "A zero standard deviation claims perfect measurement and would make the "
                  + "estimator ignore the wheels entirely (tags=" + tags
                  + ", distance=" + distance + ")");
          assertTrue(Double.isFinite(value));
        }
      }
    }
  }

  @Test
  @DisplayName("Very close tags do not produce absurdly small deviations")
  void closeRangeIsClamped() {
    // Distance factor is floored at 1.0, so a tag 10 cm away is not treated as 100x better
    // than one at 1 m — the camera calibration itself limits accuracy at that range.
    assertEquals(xyStdDev(1, 1.0), xyStdDev(1, 0.1), 1e-9);
  }

  @Test
  @DisplayName("x and y are trusted equally")
  void translationIsIsotropic() {
    Matrix<N3, N1> stdDevs = VisionSubsystem.computeStdDevs(1, 3.0);
    assertEquals(stdDevs.get(0, 0), stdDevs.get(1, 0), 1e-12);
  }

  @Test
  @DisplayName("The 2026 Rebuilt field layout loads and has sane dimensions")
  void fieldLayoutLoads() {
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(VisionConstants.FIELD_LAYOUT);

    assertTrue(layout.getFieldLength() > 10.0 && layout.getFieldLength() < 20.0,
        "Field length should be around 17 m, was " + layout.getFieldLength());
    assertTrue(layout.getFieldWidth() > 5.0 && layout.getFieldWidth() < 12.0,
        "Field width should be around 8 m, was " + layout.getFieldWidth());
    assertTrue(layout.getTags().size() > 0, "The layout must contain tags");
  }

  @Test
  @DisplayName("Every tag in the layout sits inside the field")
  void tagsAreOnTheField() {
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(VisionConstants.FIELD_LAYOUT);

    layout.getTags().forEach(tag -> {
      var pose = layout.getTagPose(tag.ID).orElseThrow();
      assertTrue(pose.getX() >= -0.5 && pose.getX() <= layout.getFieldLength() + 0.5,
          "Tag " + tag.ID + " x is off the field: " + pose.getX());
      assertTrue(pose.getY() >= -0.5 && pose.getY() <= layout.getFieldWidth() + 0.5,
          "Tag " + tag.ID + " y is off the field: " + pose.getY());
    });
  }

  @Test
  @DisplayName("Gating thresholds are internally consistent")
  void gatingThresholdsAreSane() {
    assertTrue(VisionConstants.MAX_SINGLE_TAG_AMBIGUITY > 0
            && VisionConstants.MAX_SINGLE_TAG_AMBIGUITY < 1.0,
        "Ambiguity is a 0..1 figure");
    assertTrue(VisionConstants.MAX_TAG_DISTANCE_METERS > 1.0);
    assertTrue(VisionConstants.MAX_POSE_JUMP_METERS > 0);
    assertTrue(VisionConstants.MULTI_TAG_STD_DEV_SCALE > 0
            && VisionConstants.MULTI_TAG_STD_DEV_SCALE < 1.0,
        "Multi-tag must tighten trust, not loosen it");
  }

  @Test
  @DisplayName("The camera transform is not left at the origin")
  void cameraTransformIsSet() {
    // A camera exactly at robot centre pointing level is almost certainly an unedited
    // placeholder. This will not catch a wrong measurement, only a missing one.
    var transform = VisionConstants.ROBOT_TO_CAMERA;
    boolean atOrigin = transform.getTranslation().getNorm() < 1e-9;
    assertTrue(!atOrigin,
        "ROBOT_TO_CAMERA looks unset. It must be measured on the real robot — a wrong "
            + "transform produces confidently wrong poses.");
  }

  private static double xyStdDev(int tagCount, double distance) {
    return VisionSubsystem.computeStdDevs(tagCount, distance).get(0, 0);
  }
}
