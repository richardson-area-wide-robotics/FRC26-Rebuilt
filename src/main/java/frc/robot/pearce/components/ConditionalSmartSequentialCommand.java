package frc.robot.pearce.components;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.annotation.Nullable;
import java.util.function.Supplier;

public class ConditionalSmartSequentialCommand extends SmartSequentialCommand{
    /**
     * Create a {@link SmartSequentialCommand}, allowing the robot to preform actions
     *
     * @param path   The {@link UncomputedPath} the robot should drive on
     * @param action The {@link Command} to run when the robot is finished driving, can be null
     * @param next   The next {@link SmartSequentialCommand} to run, can be null
     */
    public Supplier<Boolean> conditional;
    public SmartSequentialCommand nextSmartSequentialCommandFalse;


    public ConditionalSmartSequentialCommand(@Nullable UncomputedPath path, @Nullable Command action, @Nullable SmartSequentialCommand nextT,@Nullable SmartSequentialCommand nextF, @Nullable Supplier<Boolean> conditional) {
        super(path, action, nextT);
        this.nextSmartSequentialCommandFalse = nextF;
    }
    @Override
    public void execute(){
        SequentialCommandGroup commandToExecute = new SequentialCommandGroup(path.compute(),action);
        CommandScheduler.getInstance().schedule(commandToExecute);

        if(nextSmartSequentialCommand != null){
            if(conditional.get())nextSmartSequentialCommand.execute();
            else nextSmartSequentialCommandFalse.execute();
        }
    }
}
