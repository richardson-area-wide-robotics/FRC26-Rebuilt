package frc.robot.rebuilt.subsystems.smart;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DynamicPather {//statless

    /**
     * The Standard Constraints for a path                   
     */
    public static final PathConstraints STANDARD_CONSTRAINTS =
            new PathConstraints(
                    4.5,
                    3.5,
                    10.0,
                    20.0,
                    12.0
            );
    
    /**
     * Builds a pose offset backward from the goal along its facing direction.
     */
    public static Pose2d buildApproachPose(Pose2d goalPose, double approachDistance) {
        // Unit vector in the direction the goal is facing
        Translation2d facingDirection =
                new Translation2d(
                        goalPose.getRotation().getCos(),
                        goalPose.getRotation().getSin()
                );

        // Move backwards from the goal along its facing direction
        Translation2d approachTranslation =
                goalPose.getTranslation().minus(facingDirection.times(approachDistance));

        return new Pose2d(approachTranslation, goalPose.getRotation());
    }

    /**
     * Computes a path as a {@link Command} to a Pose2d
     * 
     * @param targetPose the pose we want to go to
     * @param pathConstraints path constraints controlling speeds
     * @param timeout time in seconds to consider the path failed and escape early                       
     */
   public static Command computePathfindCommand(Pose2d targetPose,PathConstraints pathConstraints,  double timeout){
        return AutoBuilder.pathfindToPose(targetPose, pathConstraints).withTimeout(timeout);
   }

    /**
     * Computes a path as a {@link Command} to a Pose2d, accounting for the approach
     *
     * @param targetPose the pose we want to go to
     * @param pathConstraints path constraints controlling speeds
     * @param timeout time in seconds to consider the path failed and escape early
     */
    public static Command computeApprochPathfindCommand(Pose2d targetPose,PathConstraints pathConstraints,  double timeout, double approachDistance){
        return AutoBuilder.pathfindToPose(buildApproachPose(targetPose, approachDistance), pathConstraints).withTimeout(timeout);
    }
}
