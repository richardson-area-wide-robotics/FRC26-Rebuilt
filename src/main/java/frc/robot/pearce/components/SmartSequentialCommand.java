package frc.robot.pearce.components;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.pearce.subsystems.smart.DynamicPather;

import javax.annotation.Nullable;
import java.lang.ref.Reference;
import java.util.Objects;

public class SmartSequentialCommand {

    public UncomputedPath path;
    public Command action;
    public Reference<SmartSequentialCommand> nextSmartSequentialCommand;


    public SmartSequentialCommand(@Nullable UncomputedPath path, @Nullable Command action, @Nullable Reference<SmartSequentialCommand> next){
        this.path = Objects.requireNonNullElse(path, new UncomputedPath(null,null));
        this.action = Objects.requireNonNullElse(action, Commands.none());
        this.nextSmartSequentialCommand = next;
    }

    public void execute(){
        SequentialCommandGroup commandToExecute = new SequentialCommandGroup(path.compute(),action);
        CommandScheduler.getInstance().schedule(commandToExecute);

        if(nextSmartSequentialCommand.get() != null){
            nextSmartSequentialCommand.get().execute();
        }
    }


    private static class UncomputedPath{
        public Pose2d endPose;
        public PathConstraints constraints;

        public UncomputedPath(@Nullable Pose2d endPose, @Nullable PathConstraints constraints){
            this.endPose = endPose;
            this.constraints = constraints;
        }

        public Command compute(){
            if(endPose == null) return Commands.none();
            else return DynamicPather.computePathfindCommand(endPose,constraints,20.);
        }
    }

}
