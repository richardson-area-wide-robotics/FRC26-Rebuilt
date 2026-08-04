package frc.robot.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import frc.robot.common.subsystems.vision.VisionConstants;
import frc.robot.rebuilt.RebuiltConstants.GeometryConstants;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Ties the camera transform to the shooter's mounting angle, because on this robot they are the same
 * physical axis.
 *
 * <p>The camera is mounted in line with the shooter, and the shooter is 90 degrees from the intake.
 * Those are facts about the built robot that no library and no other constant can derive, and two
 * separate places in the code depend on them:
 *
 * <ul>
 *   <li>{@code ShooterRangeModel.headingToAimShooter} needs the shooter's angle, or it aims the
 *       intake at the hub and fires the shot sideways off the field.
 *   <li>{@code VisionConstants.ROBOT_TO_CAMERA} needs the camera's angle, or every tag-derived pose
 *       comes back rotated by 90 degrees — confidently and enormously wrong.
 * </ul>
 *
 * <p>Because the camera and the shooter share an axis, those two numbers cannot legitimately
 * disagree. This asserts they do not, so moving one without the other fails the build instead of
 * failing on the field.
 */
class GeometryConsistencyTest {

  @Test
  @DisplayName("The camera yaw in ROBOT_TO_CAMERA matches the shooter's mounting angle")
  void cameraYawMatchesShooter() {
    double cameraYawDegrees =
        Units.radiansToDegrees(VisionConstants.ROBOT_TO_CAMERA.getRotation().getZ());

    assertEquals(GeometryConstants.CAMERA_YAW_OFFSET_DEGREES, cameraYawDegrees, 0.5,
        "The camera is mounted in line with the shooter, so ROBOT_TO_CAMERA's yaw must equal the "
            + "shooter offset. A yaw of 0 with a 90 degree shooter offset means every vision pose "
            + "is rotated a quarter turn.");
  }

  @Test
  @DisplayName("Camera and shooter offsets are the same number, not two copies that can drift")
  void offsetsAreOneNumber() {
    assertEquals(GeometryConstants.SHOOTER_YAW_OFFSET_DEGREES,
        GeometryConstants.CAMERA_YAW_OFFSET_DEGREES, 1e-9);
  }

  @Test
  @DisplayName("The shooter offset is a quarter turn, as built")
  void shooterOffsetIsNinetyDegrees() {
    // Magnitude is known from the robot: shooter and intake are 90 degrees apart. The sign is
    // marked CONFIRM in the constant, so only the magnitude is asserted here.
    assertEquals(90.0, Math.abs(GeometryConstants.SHOOTER_YAW_OFFSET_DEGREES), 1e-9,
        "intake and shooter are 90 degrees apart on this robot");
  }

  @Test
  @DisplayName("Camera translation is still a placeholder, and says so")
  void cameraTranslationIsUnmeasured() {
    // Not a failure — a reminder with teeth. Once the real numbers are in, this test should be
    // deleted rather than loosened, because at that point it has nothing left to protect.
    double x = VisionConstants.ROBOT_TO_CAMERA.getX();
    double z = VisionConstants.ROBOT_TO_CAMERA.getZ();

    boolean stillPlaceholder =
        Math.abs(x - Units.inchesToMeters(12.0)) < 1e-9
            && Math.abs(z - Units.inchesToMeters(8.0)) < 1e-9;

    if (stillPlaceholder) {
      System.out.println(
          "[geometry] ROBOT_TO_CAMERA translation is still the 12in/8in placeholder. "
              + "Step 0c of SHOP_RUNBOOK.md is the procedure for measuring it.");
    }

    // What must hold either way: the camera is somewhere physically possible on a 26.5in robot.
    assertTrue(Math.abs(x) < 0.6 && Math.abs(VisionConstants.ROBOT_TO_CAMERA.getY()) < 0.6,
        "camera cannot be further than 0.6 m from centre on a 26.5 in frame");
    assertTrue(z > 0 && z < 1.4, "camera height must be above the floor and under the height limit");
  }
}
