package frc.robot.common.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.common.gyro.RAWRNavX2;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class AssumedPoseSubsystem extends SubsystemBase {

    public boolean useVisionData = true;

    private final RAWRNavX2 imu;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final ModulePositionSupplier modulePositions;
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    /** Functional interface so this subsystem does not depend directly on your swerve subsystem implementation. */
    @FunctionalInterface
    public interface ModulePositionSupplier {
        SwerveModulePosition[] get();
    }

    public AssumedPoseSubsystem(
            RAWRNavX2 imu,
            SwerveDriveKinematics kinematics,
            ModulePositionSupplier modulePositions,
            Transform3d robotToCamera,
            String cameraName
    ) {
        this.imu = imu;
        this.modulePositions = modulePositions;

        photonCamera = new PhotonCamera(cameraName);

        // Initialize pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                imu.getRotation2d(),
                modulePositions.get(),
                new Pose2d(13, 4, new Rotation2d(Math.PI))
        );

        // AprilTag field layout
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);

    }

    @Override
    public void periodic() {

        List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
        if (!results.isEmpty() && useVisionData) {
            for (PhotonPipelineResult result : results) {
                Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.estimateCoprocMultiTagPose(result);
                if (estimatedPose.isPresent()) {
                    Pose2d photonPose = estimatedPose.get().estimatedPose.toPose2d();
                    Logger.recordOutput(getName() + "/PosePhoton", photonPose);

                    double poseTime = estimatedPose.get().timestampSeconds;
                    poseEstimator.addVisionMeasurement(photonPose, poseTime);
                }
            }
        }

        poseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                imu.getRotation2d(),
                modulePositions.get()
        );

        Logger.recordOutput(getName() + "/PoseAfterCompute", getPose());
    }
    /** Best assumed robot pose on the field */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Force pose reset (auto start, etc.) */
    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(
                imu.getRotation2d(),
                modulePositions.get(),
                pose
        );
    }
}
