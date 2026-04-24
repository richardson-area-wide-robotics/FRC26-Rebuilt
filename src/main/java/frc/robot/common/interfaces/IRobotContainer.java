package frc.robot.common.interfaces;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.subsystems.drive.Pose2dSupplier;

/**
 * Interface for a team's RobotContainer, each team must implement this.
 * 
 * @author Hudson Strub
 * @since 2025
 */
public interface IRobotContainer {

    /**Get the command to use during auto */
    Command getAutonomousCommand();

    /**Ran periodically during simulation */
    void simulationPeriodic();

    /**Ran periodically when the robot is disabled. (Dont try it lmao) */
    void disabledPeriodic();

    void robotInit();

    /**Ran at the start of  auto */
    void autonomousInit();

    /**Ran periodically during auto */
    void autonomousPeriodic();

    /**Ran periodically during teleop */
    void teleopPeriodic();

    /**Get the pose supplier that this robot is using for pose information */
    Pose2dSupplier getPose2dSupplier();
}
