package frc.robot.rebuilt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
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
  @DisplayName("The camera transform holds the CAD numbers, not the old placeholder")
  void cameraTransformIsFromCad() {
    double x = VisionConstants.ROBOT_TO_CAMERA.getX();
    double y = VisionConstants.ROBOT_TO_CAMERA.getY();
    double z = VisionConstants.ROBOT_TO_CAMERA.getZ();

    // From CAD, relative to the centre of the four wheel contact patches at floor level.
    assertEquals(Units.inchesToMeters(2.808), x, 1e-6);
    assertEquals(Units.inchesToMeters(6.267), y, 1e-6);
    assertEquals(Units.inchesToMeters(25.271), z, 1e-6);

    // The old placeholder, kept as an explicit negative so a revert is caught rather than merely
    // being invisible.
    assertNotEquals(Units.inchesToMeters(12.0), x, "12 in x was the placeholder");
    assertNotEquals(Units.inchesToMeters(8.0), z, "8 in z was the placeholder");

    // A positive y means the camera sits to the robot's LEFT, which is the same side the +90 yaw
    // says it looks out of. Not proof of the sign, but the two agreeing is worth asserting: a
    // camera on the left looking right would be unusual enough to want deliberate confirmation.
    assertTrue(y > 0, "CAD puts the camera left of centre");
    assertTrue(GeometryConstants.CAMERA_YAW_OFFSET_DEGREES > 0,
        "camera is left of centre and should be looking out that side; if the yaw is actually -90 "
            + "then the camera looks across the robot, which wants confirming rather than assuming");

    // Physically possible on this robot at all.
    assertTrue(Math.hypot(x, y) < 0.5,
        "camera cannot be further than 0.5 m from centre on a 26.5 in frame");
    assertTrue(z > 0 && z < 1.4, "camera height must be above the floor and under the height limit");
  }

  @Test
  @DisplayName("Camera pitch is negative, i.e. aimed up toward where tags live")
  void cameraPitchAimsUp() {
    double pitchDegrees =
        Units.radiansToDegrees(VisionConstants.ROBOT_TO_CAMERA.getRotation().getY());

    // CAD gave 5.5 degrees as a magnitude; the sign was inferred. Down would put the optical axis
    // 0.26 m off the floor at 4 m, which is carpet — tags sit around 1.2 to 1.5 m. Asserted so that
    // if someone flips it, they do so deliberately.
    assertTrue(pitchDegrees < 0,
        "pitch must be negative (aimed up). Positive aims at the floor, got " + pitchDegrees);
    assertEquals(-5.5, pitchDegrees, 0.01);
  }
}
