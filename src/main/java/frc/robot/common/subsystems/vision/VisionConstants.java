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
   * The field layout this team's field uses: AndyMark.
   *
   * <p>Welded and AndyMark fields place the tags at different coordinates — measured at up
   * to <b>3.6 cm</b> apart for the 2026 layouts. Picking the wrong one introduces a fixed
   * positional bias of that size, which looks exactly like a calibration problem and is
   * large enough to matter when lining up a shot.
   *
   * <p><b>Note for competition:</b> official event fields are normally welded. If you travel
   * to an event on a welded field, switch this to {@code k2026RebuiltWelded} — otherwise
   * every tag-derived pose carries a small constant offset, and the wheel-scale figure from
   * {@code VisionCalibration} will be quietly biased along with it.
   */
  public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2026RebuiltAndymark;

  /**
   * MEASURE — the camera name exactly as configured in the PhotonVision web UI.
   *
   * <p>A mismatch here fails silently: PhotonVision returns no results and vision simply
   * never contributes, with no error.
   */
  public static final String CAMERA_NAME = "frontCamera";

  /**
   * Degrees from chassis forward to the camera's optical axis.
   *
   * <p><b>90, because the camera is mounted in line with the shooter and the shooter is 90 degrees
   * from the intake.</b> Not zero, which is what a placeholder naturally wants to be and what this
   * originally had — and a yaw of 0 against a camera actually pointing out the side rotates every
   * tag-derived pose by a quarter turn. Vision is then confidently, enormously wrong, which is worse
   * than having no vision at all because the estimator fuses it in and reports high confidence.
   *
   * <p>Deliberately a duplicate of {@code RebuiltConstants.GeometryConstants.CAMERA_YAW_OFFSET_DEGREES}
   * rather than an import: this class sits in {@code common} and that one is this year's robot, so
   * importing it would have the shared framework depend on a specific robot. {@code
   * GeometryConsistencyTest} pins the two together instead, so they cannot drift apart silently.
   *
   * <p><b>Confirm the sign against the robot.</b> +90 means the camera looks out of the robot's left
   * side, since +y is left. Same question, and the same answer, as the shooter offset.
   */
  public static final double CAMERA_YAW_DEGREES = 90.0;

  /**
   * MEASURE — the rigid transform from robot centre to camera lens.
   *
   * <p>Translation is metres in robot coordinates: <b>+x forward, +y left, +z up</b>, measured from
   * the robot's <b>centre of rotation at floor level</b> — which is the centre of the square formed by
   * the four wheel contact patches, not the centre of the bumper perimeter. Step 0a of
   * {@code SHOP_RUNBOOK.md} is the procedure for finding and marking it.
   *
   * <p>Rotation is the camera's orientation. <b>Pitch is positive downward</b>, so a camera tilted up
   * gets a negative pitch — asserted by {@code PitchConventionTest} so it cannot be misremembered.
   *
   * <p>The yaw below is real: {@link #CAMERA_YAW_DEGREES}, because this camera looks out along the
   * shooter axis. <b>The translation and the pitch are still placeholders</b> describing a lens 12 in
   * forward of centre, on the centreline, 8 in off the floor, tilted up 15 degrees. Replace all four
   * numbers with measurements — step 0c of the runbook gives two independent methods for pitch,
   * because a few degrees there becomes tens of centimetres of pose error at the far end of the field.
   */
  public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
      new Translation3d(
          // From CAD, measured from the centre of the four wheel contact patches at floor level.
          Units.inchesToMeters(2.808),
          Units.inchesToMeters(6.267),
          Units.inchesToMeters(25.271)),
      new Rotation3d(
          0.0,
          // -5.5 deg = tilted UP by 5.5. CAD gave the magnitude; the sign is inferred from optics and
          // is worth the 30-second check in step 0d. At 4 m a downward 5.5 puts the optical axis
          // 0.26 m off the floor, which is carpet, not AprilTags. Upward puts it at 1.03 m, climbing
          // toward tag height. Down is not a plausible aim for this camera.
          Units.degreesToRadians(-5.5),
          Units.degreesToRadians(CAMERA_YAW_DEGREES)));

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
