package frc.robot.common.interfaces;

import edu.wpi.first.wpilibj2.command.Command;

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

    /**
     * Ran periodically in <em>every</em> mode — disabled, auto, teleop and test.
     *
     * <p>Use this for state that must stay current regardless of mode, such as decoding
     * FMS field state. Anything placed only in {@link #teleopPeriodic()} silently stops
     * updating during autonomous.
     */
    default void robotPeriodic() {
    }

    /**Ran at the start of  auto */
    void autonomousInit();

    /**Ran periodically during auto */
    void autonomousPeriodic();

    /**Ran periodically during teleop */
    void teleopPeriodic();

    /**
     * The robot's on-blocks self-test, run from the driver station's Test mode.
     *
     * <p>Return {@code null} if a team has no self-test. Anything returned here is scheduled
     * once when Test mode is entered.
     *
     * @return a validation command, or null.
     */
    default Command getValidationCommand() {
        return null;
    }
}
