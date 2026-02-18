package frc.robot.pearce.subsystems.smart;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.pearce.PearceContainer;
import frc.robot.pearce.components.SmartSequentialCommand;

public class SmartSectorPather {

    SmartSequentialCommand goToRedHub;

    public SmartSectorPather(){
        goToRedHub = new SmartSequentialCommand(new SmartSequentialCommand.UncomputedPath(new Pose2d(11,3,new Rotation2d()),new PathConstraints(1,1,1,1)), Commands.runOnce(PearceContainer.PROTO_SHOOTER::runShooter), null);


    }
}
