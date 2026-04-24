package frc.robot.common;

import frc.robot.common.subsystems.drive.Pose2dSupplier;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;
import frc.robot.common.interfaces.IRobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.annotations.Robot;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
@Robot(team = 0)
public class DefaultContainer implements IRobotContainer {

    public static IRobotContainer createContainer(){
        return new DefaultContainer();
    }

    @Override
    public Command getAutonomousCommand() {
        return null;
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void robotPeriodic(){

    }

    @Override
    public void robotInit() {
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public Pose2dSupplier getPose2dSupplier() {
        return null;
    }

}
