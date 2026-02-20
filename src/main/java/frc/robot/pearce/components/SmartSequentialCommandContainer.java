package frc.robot.pearce.components;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.pearce.PearceContainer;

public class SmartSequentialCommandContainer {
    private static final PathConstraints standardConstraints =
            new PathConstraints(1,1,1,1);
    //Define all usable SmartSequentialCommands here.

    public static SmartSequentialCommand goToRedHub = new SmartSequentialCommand(
            new SmartSequentialCommand.UncomputedPath(
                    new Pose2d(11,3,new Rotation2d()),
                    standardConstraints),
            null,
            null,
            "goToRedHub");
    public static SmartSequentialCommand shootInPlace = new SmartSequentialCommand(
            null,
            Commands.runOnce(PearceContainer.PROTO_SHOOTER::runShooter),
            null,
                "shootInPlace");
    public static SmartSequentialCommand returnToStart = new SmartSequentialCommand(
            new SmartSequentialCommand.UncomputedPath(new Pose2d(0,0,new Rotation2d()),
                    standardConstraints),
            null,
            null,
            "returnToStart");
    public static SmartSequentialCommand exampleComplexTask = new SmartSequentialCommand(
            new SmartSequentialCommand.UncomputedPath(new Pose2d(0,0,new Rotation2d()),
                    standardConstraints),
            Commands.runOnce(PearceContainer.SECTOR_EVALUATOR::getSector),
            returnToStart,
            "exampleComplexTask");

}
