package frc.robot.pearce.subsystems.smart;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.pearce.PearceContainer;
import frc.robot.pearce.components.SmartSequentialCommand;
import lombok.NonNull;

public class SmartSequentialCommandSequencer {
    @NonNull
    private SmartSequentialCommand rootCommand; //you cannot remove the root dont even try
    @NonNull
    private SmartSequentialCommand tailCommand;



    public SmartSequentialCommandSequencer(SmartSequentialCommand rootCommand){
        this.rootCommand = rootCommand;
        this.tailCommand = rootCommand;
    }

    public SequentialCommandGroup finalizeSequence(){
        SequentialCommandGroup commandToExecute = new SequentialCommandGroup(Commands.runOnce(()->rootCommand.updateAdvantage()),rootCommand.path.compute(),rootCommand.action);
        SmartSequentialCommand current = rootCommand.nextSmartSequentialCommand;
        while(current != null){
            commandToExecute.addCommands(Commands.runOnce(current::updateAdvantage),current.path.compute(),current.action);
            current = current.nextSmartSequentialCommand;
        }
        return commandToExecute;
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
