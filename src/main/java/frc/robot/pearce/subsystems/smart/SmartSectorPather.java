package frc.robot.pearce.subsystems.smart;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.pearce.PearceContainer;
import frc.robot.pearce.components.ConditionalSmartSequentialCommand;
import frc.robot.pearce.components.SmartSequentialCommand;
import lombok.NonNull;

import javax.annotation.Nullable;
import java.lang.ref.Reference;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

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
            null,
            null);
    public static SmartSequentialCommand shootInPlace = new SmartSequentialCommand(
            null,
            Commands.runOnce(PearceContainer.PROTO_SHOOTER::runShooter),
            null);
    public static SmartSequentialCommand loadInPlace = new SmartSequentialCommand(
            null,
            Commands.runOnce(PearceContainer.PROTO_FEEDER::load),
            null);


    //Define all usable ConditionalSmartSequentialCommands here.
    private static Boolean isLoaded(){return false;}//EXAMPLE CONDITIONAL
    public static ConditionalSmartSequentialCommand climbOrShootInPlace = new ConditionalSmartSequentialCommand(
            null,
             null,
            loadInPlace,
            shootInPlace,
            SmartSectorPather::isLoaded
    );




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
        tailCommand = Objects.requireNonNullElse(appendantCommand.nextSmartSequentialCommand, appendantCommand);
    }
    public void appendToConditional(SmartSequentialCommand appendantCommand){
        if(rootCommand == null || tailCommand == null) throw new IllegalStateException();
        if (!(tailCommand instanceof ConditionalSmartSequentialCommand)) {
            throw new IllegalStateException();
        }

        ((ConditionalSmartSequentialCommand) tailCommand).nextSmartSequentialCommandFalse = appendantCommand;
        tailCommand = appendantCommand;
    }
    public void applyConditional(Supplier<Boolean> conditional){
        if(rootCommand == null || tailCommand == null) throw new IllegalStateException();
        if (!(tailCommand instanceof ConditionalSmartSequentialCommand)) {
            throw new IllegalStateException();
        }
        ((ConditionalSmartSequentialCommand) tailCommand).conditional = conditional;

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
