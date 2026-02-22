package frc.robot.pearce.components;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.pearce.subsystems.smart.DynamicPather;

public class SmartSequentialCommandContainer {
    //Define all usable SmartSequentialCommands here.

    public static SmartSequentialCommand goToRedHub = new SmartSequentialCommand(
            new SmartSequentialCommand.UncomputedPath(
                    new Pose2d(11,3, Rotation2d.fromDegrees(180)),
                    DynamicPather.STANDARD_CONSTRAINTS),
            null,
            null,
            "goToRedHub");
    //public static SmartSequentialCommand shootInPlace = new SmartSequentialCommand(
    //        null,
    //        Commands.runOnce(PearceContainer.PROTO_SHOOTER::runShooter),
    //        null,
    //            "shootInPlace");
    //public static SmartSequentialCommand returnToStart = new SmartSequentialCommand(
    //        new SmartSequentialCommand.UncomputedPath(new Pose2d(0,0,new Rotation2d()),
    //                standardConstraints),
    //        null,
    //        null,
    //        "returnToStart");
    //public static SmartSequentialCommand exampleComplexTask = new SmartSequentialCommand(
    //        new SmartSequentialCommand.UncomputedPath(new Pose2d(0,0,new Rotation2d()),
    //                standardConstraints),
    //        Commands.runOnce(PearceContainer.SECTOR_EVALUATOR::getSector),
    //        returnToStart,
    //        "exampleComplexTask");

}
