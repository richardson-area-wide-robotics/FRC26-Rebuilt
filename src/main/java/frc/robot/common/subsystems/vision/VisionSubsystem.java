package frc.robot.common.subsystems.vision;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.diagnostics.VisionCalibration;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * AprilTag localisation: turns camera sightings into pose measurements for the drivetrain,
 * and feeds the calibration analyser at the same time.
 *
 * <p>Every accepted measurement carries its own standard deviation, scaled by how far away
 * the tags were and how many were seen. Telling the pose estimator how much to trust each
 * individual measurement is the difference between vision that tightens the estimate and
 * vision that makes it jitter.
 *
 * <p>Measurements are rejected — never fused — when they are physically implausible: off the
 * field, well off the floor, too distant, too ambiguous, or an impossible jump from the
 * current estimate. A confidently wrong pose is worse than no pose, because the estimator
 * has no way to know it should distrust it.
 *
 * <p>The subsystem is a no-op when no camera is connected, so it is safe to leave installed
 * with the camera unplugged.
 */
public class VisionSubsystem extends SubsystemBase {

    /** Accepts a vision pose with an explicit trust level. */
    @FunctionalInterface
    public interface VisionConsumer {
        /**
         * @param pose      Field-relative measured pose.
         * @param timestamp FPGA timestamp of the measurement.
         * @param stdDevs   Trust: x metres, y metres, theta radians.
         */
        void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
    }

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout;

    /** Where the active layout came from, kept so validation and telemetry can report it. */
    private final FieldLayoutLoader.Result layoutResult;

    private final VisionConsumer visionConsumer;
    private final Supplier<Pose2d> fusedPoseSupplier;
    private final Supplier<Pose2d> odometryOnlyPoseSupplier;
    private final DoubleSupplier gyroYawDegreesSupplier;
    private final DoubleSupplier chassisSpeedSupplier;

    private final VisionCalibration calibration = new VisionCalibration("Calibration");

    /** Set false to observe vision without letting it influence the pose estimate. */
    private boolean fuseIntoPoseEstimate = true;

    private int acceptedCount;
    private int rejectedCount;
    private String lastRejectReason = "";
    private double lastAcceptedTimestamp;

    /**
     * @param cameraName               PhotonVision camera name.
     * @param visionConsumer           Where accepted measurements go, normally the drivetrain.
     * @param fusedPoseSupplier        The pose estimate currently in use.
     * @param odometryOnlyPoseSupplier A wheels-and-gyro-only pose, for calibration comparison.
     * @param gyroYawDegreesSupplier   Raw gyro heading in degrees.
     * @param chassisSpeedSupplier     Robot speed in m/s, used to detect standing still.
     */
    public VisionSubsystem(
            String cameraName,
            VisionConsumer visionConsumer,
            Supplier<Pose2d> fusedPoseSupplier,
            Supplier<Pose2d> odometryOnlyPoseSupplier,
            DoubleSupplier gyroYawDegreesSupplier,
            DoubleSupplier chassisSpeedSupplier) {

        this.visionConsumer = visionConsumer;
        this.fusedPoseSupplier = fusedPoseSupplier;
        this.odometryOnlyPoseSupplier = odometryOnlyPoseSupplier;
        this.gyroYawDegreesSupplier = gyroYawDegreesSupplier;
        this.chassisSpeedSupplier = chassisSpeedSupplier;

        this.camera = new PhotonCamera(cameraName);

        // Prefer a wpical-calibrated layout if one has been deployed, otherwise the compiled-in one.
        // Which is active is logged at startup, because a practice-calibrated layout is wrong at an
        // official event and nothing else would tell you.
        this.layoutResult = FieldLayoutLoader.load();
        FieldLayoutLoader.report(layoutResult);
        this.fieldLayout = layoutResult.layout();

        this.poseEstimator =
                new PhotonPoseEstimator(fieldLayout, VisionConstants.ROBOT_TO_CAMERA);
    }

    @Override
    public void periodic() {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        Logger.recordOutput(getName() + "/Connected", camera.isConnected());
        Logger.recordOutput(getName() + "/ResultsThisLoop", results.size());
        Logger.recordOutput(getName() + "/FusingEnabled", fuseIntoPoseEstimate);

        for (PhotonPipelineResult result : results) {
            process(result);
        }

        Logger.recordOutput(getName() + "/Accepted", acceptedCount);
        Logger.recordOutput(getName() + "/Rejected", rejectedCount);
        Logger.recordOutput(getName() + "/LastRejectReason", lastRejectReason);
        Logger.recordOutput(getName() + "/SecondsSinceAccepted",
                lastAcceptedTimestamp == 0 ? -1 : Timer.getFPGATimestamp() - lastAcceptedTimestamp);

        calibration.log();
    }

    private void process(PhotonPipelineResult result) {
        Optional<EstimatedRobotPose> estimate = poseEstimator.estimateCoprocMultiTagPose(result);
        if (estimate.isEmpty()) {
            estimate = poseEstimator.estimateLowestAmbiguityPose(result);
        }
        if (estimate.isEmpty()) {
            return;
        }

        EstimatedRobotPose estimated = estimate.get();
        Pose3d pose3d = estimated.estimatedPose;
        Pose2d pose = pose3d.toPose2d();
        List<PhotonTrackedTarget> targets = estimated.targetsUsed;

        if (targets.isEmpty()) {
            reject("no targets");
            return;
        }

        double nearestTagMeters = Double.MAX_VALUE;
        double worstAmbiguity = 0;
        for (PhotonTrackedTarget target : targets) {
            nearestTagMeters = Math.min(
                    nearestTagMeters, target.getBestCameraToTarget().getTranslation().getNorm());
            worstAmbiguity = Math.max(worstAmbiguity, target.getPoseAmbiguity());
        }

        if (!isPlausible(pose3d, pose, targets.size(), nearestTagMeters, worstAmbiguity)) {
            return;
        }

        Matrix<N3, N1> stdDevs = computeStdDevs(targets.size(), nearestTagMeters);

        acceptedCount++;
        lastAcceptedTimestamp = Timer.getFPGATimestamp();

        Logger.recordOutput(getName() + "/MeasuredPose", pose);
        Logger.recordOutput(getName() + "/TagCount", targets.size());
        Logger.recordOutput(getName() + "/NearestTagMeters", nearestTagMeters);
        Logger.recordOutput(getName() + "/WorstAmbiguity", worstAmbiguity);
        Logger.recordOutput(getName() + "/StdDevXY", stdDevs.get(0, 0));
        Logger.recordOutput(getName() + "/StdDevThetaDeg",
                Math.toDegrees(stdDevs.get(2, 0)));

        double latency = Timer.getFPGATimestamp() - estimated.timestampSeconds;

        // Calibration always sees the measurement, even when fusing is switched off — that
        // is how you evaluate vision quality without letting it move the robot.
        calibration.addSample(
                pose,
                odometryOnlyPoseSupplier.get(),
                fusedPoseSupplier.get(),
                gyroYawDegreesSupplier.getAsDouble(),
                nearestTagMeters,
                latency,
                chassisSpeedSupplier.getAsDouble());

        if (fuseIntoPoseEstimate) {
            visionConsumer.accept(pose, estimated.timestampSeconds, stdDevs);
        }
    }

    /** Applies every plausibility gate, recording why a measurement was thrown away. */
    private boolean isPlausible(
            Pose3d pose3d, Pose2d pose, int tagCount, double nearestTagMeters, double ambiguity) {

        if (nearestTagMeters > VisionConstants.MAX_TAG_DISTANCE_METERS) {
            reject("tag too far: " + nearestTagMeters);
            return false;
        }

        if (tagCount == 1 && ambiguity > VisionConstants.MAX_SINGLE_TAG_AMBIGUITY) {
            reject("ambiguous single tag: " + ambiguity);
            return false;
        }

        if (Math.abs(pose3d.getZ()) > VisionConstants.MAX_Z_ERROR_METERS) {
            reject("off the floor: z=" + pose3d.getZ());
            return false;
        }

        double margin = VisionConstants.FIELD_MARGIN_METERS;
        if (pose.getX() < -margin
                || pose.getY() < -margin
                || pose.getX() > fieldLayout.getFieldLength() + margin
                || pose.getY() > fieldLayout.getFieldWidth() + margin) {
            reject("off field: " + pose.getTranslation());
            return false;
        }

        // A single tag claiming a large jump is more likely a misread ID than a real
        // teleport. Multi-tag solutions are trusted to correct large errors, which is what
        // makes recovering from a bad starting pose possible.
        if (tagCount == 1) {
            double jump = pose.getTranslation()
                    .getDistance(fusedPoseSupplier.get().getTranslation());
            if (jump > VisionConstants.MAX_POSE_JUMP_METERS) {
                reject("implausible jump: " + jump);
                return false;
            }
        }

        return true;
    }

    private void reject(String reason) {
        rejectedCount++;
        lastRejectReason = reason;
    }

    /**
     * Trust model: error grows with the square of distance and shrinks with tag count.
     *
     * @param tagCount         How many tags contributed.
     * @param nearestTagMeters Distance to the closest contributing tag.
     * @return standard deviations as x metres, y metres, theta radians.
     */
    public static Matrix<N3, N1> computeStdDevs(int tagCount, double nearestTagMeters) {
        double distanceFactor = Math.max(1.0, nearestTagMeters * nearestTagMeters);
        double countFactor = Math.max(1, tagCount);

        double xy = VisionConstants.SINGLE_TAG_XY_STD_DEV_BASE * distanceFactor / countFactor;
        double theta =
                VisionConstants.SINGLE_TAG_THETA_STD_DEV_BASE * distanceFactor / countFactor;

        if (tagCount > 1) {
            xy *= VisionConstants.MULTI_TAG_STD_DEV_SCALE;
            theta *= VisionConstants.MULTI_TAG_STD_DEV_SCALE;
        }

        return VecBuilder.fill(xy, xy, theta);
    }

    /**
     * Enables or disables fusing vision into the pose estimate.
     *
     * <p>Disable to gather calibration data without vision influencing the robot — useful
     * when you do not yet trust the camera transform.
     *
     * @param fuse true to fuse measurements.
     */
    public void setFuseIntoPoseEstimate(boolean fuse) {
        this.fuseIntoPoseEstimate = fuse;
    }

    /** @return the calibration analyser, for reporting and for the validation suite. */
    public VisionCalibration getCalibration() {
        return calibration;
    }

    /** @return true when the camera is talking to us. */
    public boolean isConnected() {
        return camera.isConnected();
    }

    /** @return true when a measurement has been accepted recently. */
    public boolean hasRecentMeasurement() {
        return lastAcceptedTimestamp > 0
                && Timer.getFPGATimestamp() - lastAcceptedTimestamp < 1.0;
    }

    /** @return how many measurements have been fused. */
    public int getAcceptedCount() {
        return acceptedCount;
    }

    /** @return how many measurements have been thrown away by the plausibility gates. */
    public int getRejectedCount() {
        return rejectedCount;
    }

    /** @return the field layout in use, for tests and diagnostics. */
    /**
     * @return where the active field layout came from.
     *
     *     <p>Worth asserting on before an event: {@code isCalibrated()} being true means the robot is
     *     using measurements of the practice field, which an official field does not share.
     */
    public FieldLayoutLoader.Result getLayoutResult() {
        return layoutResult;
    }

    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }
}
