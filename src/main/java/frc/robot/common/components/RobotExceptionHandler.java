package frc.robot.common.components;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Prevents the robot from crashing after a exception
 * NOT a get out of jail free card, things are definitely borked
 *
 * Honestly im not sure why this doesn't work
 *
 * @author Hudson Strub
 * @since 2025
 */
public class RobotExceptionHandler implements Thread.UncaughtExceptionHandler {
    @Override
    public void uncaughtException(Thread t, Throwable e) {
        // Log the exception
        DriverStation.reportError("Uncaught exception in thread: " + t.getName(), e.getStackTrace());
    }
}

