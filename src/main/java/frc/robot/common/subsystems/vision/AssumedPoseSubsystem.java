package frc.robot.common.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.common.gyro.RAWRNavX2;

public class AssumedPoseSubsystem extends SubsystemBase {

    private final RAWRNavX2 imu;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;

    private final SwerveDriveKinematics kinematics;
    private final ModulePositionSupplier modulePositions;

    /**
     * Functional interface so this subsystem does not depend directly
     * on your swerve subsystem implementation.
     */
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
        this.kinematics = kinematics;
        this.modulePositions = modulePositions;

        poseEstimator =
                new SwerveDrivePoseEstimator(
                        kinematics,
                        imu.getRotation2d(),
                        modulePositions.get(),
                        new Pose2d()
                );

        // Vision trust tuning (meters, meters, radians)
        poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(0.5, 0.5, Math.toRadians(10))
        );

        camera = new PhotonCamera(cameraName);

        photonEstimator =
                new PhotonPoseEstimator(
                        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        robotToCamera
                );

        photonEstimator.setMultiTagFallbackStrategy(
                PoseStrategy.LOWEST_AMBIGUITY
        );
    }

    @Override
    public void periodic() {
        // 1) Update odometry from gyro + wheels
        poseEstimator.update(
                imu.getRotation2d(),
                modulePositions.get()
        );

        // 2) Feed gyro heading into PhotonVision
        photonEstimator.addHeadingData(
                Timer.getFPGATimestamp(),
                imu.getRotation2d()
        );

        // 3) Give PhotonVision a reference pose
        photonEstimator.setReferencePose(
                new Pose3d(poseEstimator.getEstimatedPosition())
        );

        // 4) Get vision estimate
        Optional<EstimatedRobotPose> visionEstimate =
                photonEstimator.update(camera.getLatestResult());

        // 5) Fuse vision if valid
        visionEstimate.ifPresent(estimate -> {
            poseEstimator.addVisionMeasurement(
                    estimate.estimatedPose.toPose2d(),
                    estimate.timestampSeconds
            );
        });
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
