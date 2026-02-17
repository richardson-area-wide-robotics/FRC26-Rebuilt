package frc.robot.pearce.subsystems.smart;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DynamicPather {//statless

    /**
     * Builds a pose offset backward from the goal along its facing direction.
     */
    public static Pose2d buildApproachPose(Pose2d robotPose, Pose2d goalPose, double approachDistance) {
        // Vector from robot to goal
        Translation2d delta = goalPose.getTranslation().minus(robotPose.getTranslation());
        double distance = delta.getNorm();

        // Avoid division by zero
        Translation2d direction = distance > 1e-6 ? delta.times(1.0 / distance) : new Translation2d();

        // Move back from the goal along this line
        Translation2d approachTranslation = goalPose.getTranslation().minus(direction.times(approachDistance));

        // Keep the goal rotation
        return new Pose2d(approachTranslation, goalPose.getRotation());
    }

   public static Command computePathfindCommand(Pose2d targetPose,PathConstraints pathConstraints,  double timeout){

       return AutoBuilder.pathfindToPose(targetPose, pathConstraints)
               .withTimeout(timeout);

   }
}
