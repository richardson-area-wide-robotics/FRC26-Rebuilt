package frc.robot.common.interfaces;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Interface for a team's RobotContainer, 
 * ideally each team that needs different functionality would implement this
 * 
 * @author Hudson Strub
 * @since 2025
 */
public interface IRobotContainer {

    static IRobotContainer createContainer() {
        throw new UnsupportedOperationException("createContainer must be implemented in the specific RobotContainer class");
    }

    /**Get the command to use during auto */
    Command getAutonomousCommand();

    /**Ran periodically during simulation */
    void simulationPeriodic();

    /**Ran periodically when the robot is disabled. (Dont try it lmao) */
    void disabledPeriodic();

    /**Ran periodically during auto */
    void autonomousPeriodic();

    /**Ran periodically during teleop */
    void teleopPeriodic();
}
