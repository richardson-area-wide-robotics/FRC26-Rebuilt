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
import org.lasarobotics.vision.AprilTagCamera;

public class AssumedPoseSubsystem extends SubsystemBase {

    private final RAWRNavX2 imu;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final AprilTagCamera aprilTagCamera;
    private final ModulePositionSupplier modulePositions;

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

        // Initialize pose estimator
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                imu.getRotation2d(),
                modulePositions.get(),
                new Pose2d(13, 4, new Rotation2d(Math.PI))
        );

        // AprilTag field layout
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // AprilTag camera wrapper
        aprilTagCamera = new AprilTagCamera(
                cameraName,
                imu,
                robotToCamera,
                AprilTagCamera.Resolution.RES_1280_800,
                Rotation2d.fromDegrees(89.4),
                fieldLayout
        );
    }

    @Override
    public void periodic() {
        // Only update odometry from gyro + wheels every loop
        poseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                imu.getRotation2d(),
                modulePositions.get()
        );
    }

    /**
     * Fuse the latest available AprilTag pose into the estimator.
     * Only call this when you want to update from vision.
     */
    public void fuseLatestVisionPose() {
        AprilTagCamera.Result visionResult = aprilTagCamera.getLatestEstimatedPose();

        if (visionResult != null) {
            Pose2d newPose = visionResult.estimatedRobotPose.estimatedPose.toPose2d();

            // Weak vision heading to prevent orbiting
            var stdDevs = visionResult.standardDeviation.copy();
            stdDevs.set(2, 0, Math.toRadians(999)); // X/Y strong, theta weak

            // Add vision measurement
            poseEstimator.addVisionMeasurement(
                    newPose,
                    visionResult.estimatedRobotPose.timestampSeconds,
                    stdDevs
            );
        }
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
