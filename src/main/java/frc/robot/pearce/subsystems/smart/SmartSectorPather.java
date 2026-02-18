package frc.robot.pearce.subsystems.smart;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.pearce.PearceContainer;
import frc.robot.pearce.components.SmartSequentialCommand;
import lombok.NonNull;

import java.lang.ref.Reference;
import java.util.List;

public class SmartSectorPather {
    @NonNull
    private SmartSequentialCommand rootCommand; //you cannot remove the root dont even try
    @NonNull
    private SmartSequentialCommand tailCommand;

    private static PathConstraints standardConstraints =
            new PathConstraints(1,1,1,1);
    //Define all usable SmartSequentialCommands here.

    public static SmartSequentialCommand goToRedHub = new SmartSequentialCommand(
            new SmartSequentialCommand.UncomputedPath(
                new Pose2d(11,3,new Rotation2d()),
                standardConstraints),
            Commands.none(),
            null);
    public static SmartSequentialCommand shootInPlace = new SmartSequentialCommand(
            new SmartSequentialCommand.UncomputedPath(
            null,
            null),
            Commands.runOnce(PearceContainer.PROTO_SHOOTER::runShooter),
            null);


    public SmartSectorPather(SmartSequentialCommand rootCommand){
        this.rootCommand = rootCommand;
        this.tailCommand = rootCommand;
    }

    public void executeSequence(){
        rootCommand.execute();
    }


    public void appendNode(SmartSequentialCommand appendantCommand){
        if(rootCommand == null || tailCommand == null) throw new IllegalStateException();


        tailCommand.nextSmartSequentialCommand = appendantCommand;
        tailCommand = appendantCommand;
    }
    public void removeNode(){
        if (rootCommand == null) throw new IllegalStateException();

        if (rootCommand.nextSmartSequentialCommand == null) {
            return;
        }

        SmartSequentialCommand deepestCommand = rootCommand;

        while (deepestCommand.nextSmartSequentialCommand.nextSmartSequentialCommand != null) {
            deepestCommand = deepestCommand.nextSmartSequentialCommand;
        }
        deepestCommand.nextSmartSequentialCommand = null;
        tailCommand = deepestCommand;

    }


}
