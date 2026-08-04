package frc.robot.probe;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Pins WPILib's pitch sign convention, because the runbook tells a human which way to point a
 * camera and getting that backwards puts every tag-derived pose on the wrong side of the field.
 *
 * <p>Reasoning it out: +x forward, +y left, +z up is right-handed, so a positive rotation about +y
 * carries +z toward +x — the nose goes down. That makes a camera tilted UP a NEGATIVE pitch. This
 * asserts it rather than trusting the reasoning.
 */
class PitchConventionTest {

  @Test
  @DisplayName("Negative pitch points the camera UP, which is what the runbook tells people")
  void negativePitchIsTiltedUp() {
    Translation3d forwardAxis =
        new Translation3d(1, 0, 0).rotateBy(new Rotation3d(0, Math.toRadians(-15), 0));

    assertTrue(forwardAxis.getZ() > 0,
        "A -15 degree pitch must aim the camera's forward axis upward (+z), got z="
            + forwardAxis.getZ());
  }

  @Test
  @DisplayName("Positive pitch points the camera DOWN")
  void positivePitchIsTiltedDown() {
    Translation3d forwardAxis =
        new Translation3d(1, 0, 0).rotateBy(new Rotation3d(0, Math.toRadians(15), 0));

    assertTrue(forwardAxis.getZ() < 0,
        "A +15 degree pitch must aim the camera downward, got z=" + forwardAxis.getZ());
  }
}
