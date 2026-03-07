package frc.robot.rebuilt.components;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.RebuiltContainer;
import frc.robot.rebuilt.subsystems.smart.DynamicPather;
import org.littletonrobotics.junction.Logger;

import javax.annotation.Nullable;
import java.util.Objects;

public class SmartSequentialCommand {

    public UncomputedPath path;
    public Command action;
    public SmartSequentialCommand nextSmartSequentialCommand;
    public String commandName;

    public void updateAdvantage() {
        var currentPose = RebuiltContainer.DRIVE_SUBSYSTEM.getPose();

        Logger.recordOutput("Assist/currentCommand", commandName);
        Logger.recordOutput("Assist/pathEndPose", path.endPose);
        Logger.recordOutput("Assist/startPose", currentPose);

        if (path.endPose != null) {
            double distanceMeters =
                    currentPose.getTranslation().getDistance(path.endPose.getTranslation());

            Logger.recordOutput("Assist/distanceToTargetMeters", distanceMeters);

            // rotation error
            double rotationErrorDeg =
                    path.endPose.getRotation()
                            .minus(currentPose.getRotation())
                            .getDegrees();

            Logger.recordOutput("Assist/rotationErrorDeg", rotationErrorDeg);
        }
    }

    /**
     * Create a {@link SmartSequentialCommand}, allowing the robot to preform actions
     *
     * @param path The {@link UncomputedPath} the robot should drive on
     * @param action The {@link Command} to run when the robot is finished driving, can be null
     * @param next The next {@link SmartSequentialCommand} to run, can be null
     */
    public SmartSequentialCommand(@Nullable UncomputedPath path, @Nullable Command action, @Nullable SmartSequentialCommand next,@Nullable String name){
        this.path = Objects.requireNonNullElse(path, new UncomputedPath(null,null));
        this.action = Objects.requireNonNullElse(action, Commands.none());
        this.nextSmartSequentialCommand = next;
        this.commandName = name;
    }

    /**
     * Run the {@link SmartSequentialCommand}. This builds the path using the current robot pose, runs that path, and executes the next {@link SmartSequentialCommand}
     */


    public static class UncomputedPath{
        public Pose2d endPose;
        public PathConstraints constraints;

        /**
         * Create a {@link SmartSequentialCommand}, allowing us to path at runtime
         *
         * @param endPose The pose our robot wants to end up at, can be null (if we dont want to move)
         * @param constraints The path constraints to control how fast we move, can be null (But only if {@code endPose} is null)
         */
        public UncomputedPath(@Nullable Pose2d endPose, @Nullable PathConstraints constraints){
            this.endPose = endPose;
            this.constraints = constraints;
        }

        /**
         * Build the path into a command we can run
         */
        public Command compute(){
            if (endPose == null) return Commands.none();

            Pose2d currentPose = RebuiltContainer.DRIVE_SUBSYSTEM.getPose();
            double distance = currentPose.getTranslation().getDistance(endPose.getTranslation());
            double rotationDiff = Math.abs(
                    currentPose.getRotation().minus(endPose.getRotation()).getRadians()
            );

            // CRITICAL: Prevent malformed spline generation
            if (distance < 0.05) {
                // Too close to pathfind safely
                // Just rotate or do nothing instead
                if (rotationDiff < Math.toRadians(5)) {
                    return Commands.none(); // Already there
                }

                // Only rotate in place instead of pathfinding
                return Commands.runOnce(() ->
                        RebuiltContainer.DRIVE_SUBSYSTEM.resetPose(
                                new Pose2d(currentPose.getTranslation(), endPose.getRotation())
                        )
                );
            }

            return DynamicPather.computePathfindCommand(endPose, constraints, 20);
        }
    }

}
