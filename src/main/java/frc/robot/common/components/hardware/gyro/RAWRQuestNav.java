package frc.robot.common.components.hardware.gyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity;

import java.time.Duration;
import java.time.Instant;

import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;

/**
 * QuestNav implementation of the IMU interface.
 *
 * @author Hudson Strub
 * @since 2025 Offseason
 */
public class RAWRQuestNav implements IMU {
    private final QuestNav questNav;

    // Robot-to-Quest transform (must be set for your mounting position!)
    private final Transform2d ROBOT_TO_QUEST;

    // For velocity + yaw rate calculation
    private Pose2d lastPose = new Pose2d();
    private Instant lastTime = Instant.now();

    public RAWRQuestNav(Transform2d robotToQuest) {
        this.questNav = new QuestNav();
        this.ROBOT_TO_QUEST = robotToQuest;
    }

    @Override
    public void updateInputs() {
        // Required by QuestNav each periodic cycle
        questNav.commandPeriodic();
    }

    @Override
    public Frequency getUpdateRate() {
        // QuestNav typically runs at 120Hz
        return Units.Hertz.of(120.0);
    }

    @Override
    public boolean isConnected() {
        return questNav.isConnected();
    }

    @Override
    public boolean isCalibrating() {
        return !questNav.isTracking(); // TODO this correct?
    }

    @Override
    public void reset() {
        // Reset pose to zero (field origin) — adjust if you need field-relative reset
        questNav.setPose(new Pose3d());
    }

    /** Latest robot pose (after applying ROBOT_TO_QUEST transform). */
    private Pose2d getRobotPose() {
        PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
        if (frames.length == 0) {
            return lastPose;
        }
        Pose2d questPose = frames[frames.length - 1].questPose3d().toPose2d();
        Pose2d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
        lastPose = robotPose;
        lastTime = Instant.now();
        return robotPose;
    }

    @Override
    public Angle getRoll() {
        // QuestNav provides 2D pose only → roll not available
        return Units.Radians.zero();
    }

    @Override
    public Angle getPitch() {
        // QuestNav provides 2D pose only → pitch not available
        return Units.Radians.zero();
    }

    @Override
    public Angle getYaw() {
        return Units.Radians.of(getRobotPose().getRotation().getRadians());
    }

    @Override
    public AngularVelocity getYawRate() {
        Pose2d pose = getRobotPose();
        Instant now = Instant.now();
        double dt = Duration.between(lastTime, now).toNanos() / 1e9;
        double rate = (pose.getRotation().getRadians() - lastPose.getRotation().getRadians()) / dt;
        return Units.RadiansPerSecond.of(rate);
    }

    @Override
    public Rotation2d getRotation2d() {
        return getRobotPose().getRotation();
    }

    @Override
    public LinearVelocity getVelocityX() {
        Pose2d pose = getRobotPose();
        Instant now = Instant.now();
        double dt = Duration.between(lastTime, now).toNanos() / 1e9;
        double vx = (pose.getX() - lastPose.getX()) / dt;
        return Units.MetersPerSecond.of(vx);
    }

    @Override
    public LinearVelocity getVelocityY() {
        Pose2d pose = getRobotPose();
        Instant now = Instant.now();
        double dt = Duration.between(lastTime, now).toNanos() / 1e9;
        double vy = (pose.getY() - lastPose.getY()) / dt;
        return Units.MetersPerSecond.of(vy);
    }

    @Override
    public void updateSim(Rotation2d orientation, ChassisSpeeds desiredSpeeds,
                          ControlCentricity controlCentricity) {
        // QuestNav does not simulate in WPILib; no-op
    }

    @Override
    public void close() {
        // No explicit shutdown required
    }
}
