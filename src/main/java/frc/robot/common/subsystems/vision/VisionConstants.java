package frc.robot.common.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * Everything the AprilTag pipeline needs to know about this robot's cameras and this field.
 *
 * <p><strong>Three of these values must be checked against the real hardware before vision
 * is trusted.</strong> They are marked MEASURE below. Nothing in the code can derive them,
 * and a wrong camera transform produces confidently wrong poses — worse than no vision at
 * all, because the pose estimator will fuse the error in.
 */
public final class VisionConstants {

  private VisionConstants() {
  }

  /**
   * MEASURE — the field layout your field actually uses.
   *
   * <p>Welded and AndyMark fields place the tags at slightly different coordinates. Most
   * competition fields are welded; many practice fields built from the AndyMark kit are not.
   * Using the wrong one introduces a fixed positional bias of a few centimetres that will
   * look like a calibration problem and waste an afternoon.
   */
  public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2026RebuiltWelded;

  /**
   * MEASURE — the camera name exactly as configured in the PhotonVision web UI.
   *
   * <p>A mismatch here fails silently: PhotonVision returns no results and vision simply
   * never contributes, with no error.
   */
  public static final String CAMERA_NAME = "frontCamera";

  /**
   * MEASURE — the rigid transform from robot centre to camera lens.
   *
   * <p>Translation is metres in robot coordinates: +x forward, +y left, +z up, measured from
   * the robot's centre of rotation on the floor. Rotation is the camera's orientation: pitch
   * is positive downward in this constructor's convention, so a camera tilted up gets a
   * negative pitch.
   *
   * <p>The placeholder below describes a camera 30 cm forward of centre, on the centreline,
   * 20 cm off the floor, pointing straight ahead and tilted up 15 degrees. <b>Replace every
   * number with a measurement from the actual robot.</b> An error of a few degrees in pitch
   * translates to tens of centimetres of pose error at the far end of the field.
   */
  public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
      new Translation3d(
          Units.inchesToMeters(12.0),
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(8.0)),
      new Rotation3d(
          0.0,
          Units.degreesToRadians(-15.0),
          0.0));

  /**
   * Reject any single-tag sighting whose pose ambiguity exceeds this.
   *
   * <p>A lone tag viewed nearly edge-on has two mathematically valid solutions; PhotonVision
   * reports how ambiguous the choice was. Anything above roughly 0.2 is a coin flip and will
   * teleport the pose estimate across the field if fused.
   */
  public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.2;

  /**
   * Ignore tags further away than this, in metres.
   *
   * <p>Error grows with the square of distance, so distant tags contribute mostly noise.
   */
  public static final double MAX_TAG_DISTANCE_METERS = 6.0;

  /**
   * Reject a measurement that would move the pose estimate further than this in one step,
   * in metres, unless several tags agree.
   *
   * <p>Guards against a mis-decoded tag ID putting the robot on the wrong side of the field.
   */
  public static final double MAX_POSE_JUMP_METERS = 1.5;

  /** Tolerance, in metres, for accepting a pose slightly outside the field perimeter. */
  public static final double FIELD_MARGIN_METERS = 0.5;

  /** Reject poses more than this far above or below the floor, in metres. */
  public static final double MAX_Z_ERROR_METERS = 0.4;

  /**
   * Baseline translational standard deviation for a single tag at 1 m, in metres.
   *
   * <p>Scaled by distance squared and divided by the tag count at runtime. Start here, then
   * replace with the value {@code Calibration/VisionNoise/MeasuredXyStdDevMeters} reports
   * after a stationary run — that figure is measured rather than assumed.
   */
  public static final double SINGLE_TAG_XY_STD_DEV_BASE = 0.35;

  /** Baseline rotational standard deviation for a single tag at 1 m, in radians. */
  public static final double SINGLE_TAG_THETA_STD_DEV_BASE = Units.degreesToRadians(20.0);

  /**
   * How much more to trust a multi-tag solution.
   *
   * <p>A multi-tag solve is geometrically constrained in a way a single tag is not, so its
   * error is far smaller.
   */
  public static final double MULTI_TAG_STD_DEV_SCALE = 0.25;
}
