package frc.robot.pearce.subsystems.smart;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.pearce.PearceContainer;
import frc.robot.pearce.components.HubStatus;
import lombok.Getter;

import java.util.function.Supplier;

public class TeleopAssistSubsystem extends SubsystemBase {

    private final SwerveDriveSubsystem drive;

    @Getter
    private boolean enabled = false;

    @Getter
    private HubStatus.HubState cachedHubState = HubStatus.HubState.ACTIVE;

    public enum Goal {
        NONE(null, 0.0),

        /** Example fixed pose on the field */
        EXAMPLE_FIXED(
                () -> new Pose2d(
                        new Translation2d(7, 4.1),
                        Rotation2d.fromDegrees(180)
                ),
                1.5
        ),

        /** Vision-driven / dynamically-updated pose */
        DYNAMIC(
                () -> dynamicTarget,
                1.5
        );

        private final Supplier<Pose2d> poseSupplier;
        @Getter private final double approachDistance;

        Goal(Supplier<Pose2d> poseSupplier, double approachDistance) {
            this.poseSupplier = poseSupplier;
            this.approachDistance = approachDistance;
        }

        public Pose2d getPose() {
            return poseSupplier != null ? poseSupplier.get() : new Pose2d();
        }

    }

    /** Backing store for dynamic goals (vision, cursor, etc.) */
    private static Pose2d dynamicTarget;

    public void setDynamicTarget(Pose2d pose) {
        dynamicTarget = pose;
    }

    @Getter
    private Goal currentGoal = Goal.NONE;

    public void setGoal(Goal goal) {
        if (goal == currentGoal) return;
        cancelAI();
        currentGoal = goal;
    }

    public boolean hasTarget() {
        return currentGoal != Goal.NONE && currentGoal.getPose() != null;
    }

    private Command activeCommand;

    private enum AssistState {
        DISABLED,
        IDLE,
        ASSISTING,
        HOLDING,
        SUSPENDED
    }

    @Getter
    private AssistState state = AssistState.DISABLED;

    private Pose2d lastGoalPose;
    private double lastPlanTime = 0.0;

    private static final PathConstraints AI_CONSTRAINTS =
            new PathConstraints(
                    7.0,
                    3.0,
                    Math.toRadians(540),
                    Math.toRadians(720));

    private static final double HOLD_DISTANCE = 0.75;
    private static final double REPLAN_DISTANCE = 0.4;
    private static final double REPLAN_COOLDOWN = 0.5;

    public TeleopAssistSubsystem(SwerveDriveSubsystem drive) {
        this.drive = drive;
        this.setGoal(Goal.EXAMPLE_FIXED);
    }

    public void enable() {
        enabled = true;
        transitionTo(AssistState.IDLE);
    }

    public void disable() {
        enabled = false;
        cancelAI();
        currentGoal = Goal.NONE;
        transitionTo(AssistState.DISABLED);
    }

    public void toggle() {
        if (enabled) disable();
        else enable();
    }

    public boolean isRunning() {
        return activeCommand != null && activeCommand.isScheduled();
    }

    public boolean isHubUsable() {
        double matchTime = Timer.getMatchTime();
        var alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) return false;

        cachedHubState = HubStatus.getHubStatus(alliance.get(), matchTime);
        return cachedHubState == HubStatus.HubState.ACTIVE;
    }

    private void perform() {
        if (!enabled) {
            transitionTo(AssistState.DISABLED);
            return;
        }

        if (!hasTarget()) {
            cancelAI();
            transitionTo(AssistState.IDLE);
            return;
        }

        if (!isHubUsable()) {
            cancelAI();
            transitionTo(AssistState.SUSPENDED);
            return;
        }

        Pose2d goalPose = currentGoal.getPose();
        Pose2d robotPose = drive.getPose();

        double distanceToTarget =
                robotPose.getTranslation()
                        .getDistance(goalPose.getTranslation());

        if (distanceToTarget < HOLD_DISTANCE) {
            cancelAI();
            transitionTo(AssistState.HOLDING);
            return;
        }

        Pose2d approachPose =
                buildApproachPose(goalPose, currentGoal.getApproachDistance());

        boolean shouldReplan =
                !isRunning()
                        || lastGoalPose == null
                        || robotPose.getTranslation()
                        .getDistance(lastGoalPose.getTranslation()) > REPLAN_DISTANCE
                        || Timer.getFPGATimestamp() - lastPlanTime > REPLAN_COOLDOWN;

        if (!shouldReplan) return;

        cancelAI();

        activeCommand =
                AutoBuilder.pathfindToPose(approachPose, AI_CONSTRAINTS)
                        .withTimeout(3.5);

        lastGoalPose = approachPose;
        lastPlanTime = Timer.getFPGATimestamp();
        activeCommand.schedule();

        transitionTo(AssistState.ASSISTING);
    }

    private void cancelAI() {
        if (activeCommand != null) {
            activeCommand.cancel();
            activeCommand = null;
        }
    }

    private void transitionTo(AssistState newState) {
        if (state == newState) return;
        state = newState;
    }

    /**
     * Builds a pose offset backward from the goal along its facing direction.
     */
    private Pose2d buildApproachPose(Pose2d goalPose, double approachDistance) {
        Rotation2d facing = goalPose.getRotation();

        Translation2d offset =
                new Translation2d(
                        Math.cos(facing.getRadians()),
                        Math.sin(facing.getRadians()))
                        .times(approachDistance);

        Translation2d approachTranslation =
                goalPose.getTranslation().minus(offset);

        return new Pose2d(approachTranslation, facing);
    }

    @Override
    public void periodic() {
        perform();

        PearceContainer.DRIVE_SUBSYSTEM.FIELD.getObject("TARGET").setPose(currentGoal.getPose());

        SmartDashboard.putBoolean("AI Enabled", enabled);
        SmartDashboard.putBoolean("AI Running", isRunning());
        SmartDashboard.putString("Assist State", state.name());
        SmartDashboard.putString("Goal", currentGoal.name());
        SmartDashboard.putString("Hub State", cachedHubState.name());

        if (hasTarget()) {
            SmartDashboard.putNumber(
                    "Distance To Target",
                    drive.getPose()
                            .getTranslation()
                            .getDistance(currentGoal.getPose().getTranslation()));
        }
    }
}
