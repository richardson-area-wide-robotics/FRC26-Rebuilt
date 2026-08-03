package frc.robot.testutil;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Shared setup for tests that need the WPILib HAL.
 *
 * <p>Anything touching {@code DriverStation}, {@code Timer}, a motor controller or a
 * subsystem needs the HAL running. This requires {@code includeDesktopSupport = true} in
 * build.gradle — with it off, none of these tests can exist, which is why the drivetrain bug
 * went unnoticed.
 */
public final class HalFixture {

  private HalFixture() {
  }

  /** Brings up the HAL. Safe to call repeatedly; call from {@code @BeforeAll}. */
  public static void initialize() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
  }

  /**
   * Puts the driver station into a known, enabled, teleop state on a given alliance.
   *
   * @param red true for red alliance, false for blue.
   */
  public static void enableTeleop(boolean red) {
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setAllianceStationId(
        red
            ? edu.wpi.first.hal.AllianceStationID.Red1
            : edu.wpi.first.hal.AllianceStationID.Blue1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
  }

  /** Detaches the driver station so no alliance is reported. */
  public static void clearAlliance() {
    DriverStationSim.setAllianceStationId(edu.wpi.first.hal.AllianceStationID.Unknown);
    DriverStationSim.setDsAttached(false);
    DriverStationSim.notifyNewData();
  }

  /** Sets the FMS game-specific message and pushes it through. */
  public static void setGameData(String message) {
    DriverStationSim.setGameSpecificMessage(message);
    DriverStationSim.notifyNewData();
  }

  /**
   * Simulates a driver joystick axis.
   *
   * @param axis  Axis index. Xbox: 0 left X, 1 left Y, 4 right X.
   * @param value Axis value, −1..1.
   */
  public static void setDriverAxis(int axis, double value) {
    DriverStationSim.setJoystickAxisCount(0, 6);
    DriverStationSim.setJoystickAxis(0, axis, value);
    DriverStationSim.notifyNewData();
  }

  /** Centres every driver axis. */
  public static void centreDriverSticks() {
    DriverStationSim.setJoystickAxisCount(0, 6);
    for (int axis = 0; axis < 6; axis++) {
      DriverStationSim.setJoystickAxis(0, axis, 0.0);
    }
    DriverStationSim.notifyNewData();
  }

  /**
   * Runs the command scheduler a number of times, as the robot loop would.
   *
   * @param iterations How many scheduler passes to run.
   */
  public static void runScheduler(int iterations) {
    for (int i = 0; i < iterations; i++) {
      CommandScheduler.getInstance().run();
    }
  }

  /**
   * Cancels every scheduled command, leaving subsystem registration intact.
   *
   * <p>Deliberately does <em>not</em> call {@code unregisterAllSubsystems()}: an unregistered
   * subsystem's default command never runs, which silently makes any default-command test
   * pass or fail for the wrong reason.
   */
  public static void resetScheduler() {
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * Returns a subsystem to a clean state for a fresh test: nothing scheduled, no default
   * command, still registered with the scheduler.
   *
   * @param subsystem Subsystem to reset.
   */
  public static void resetSubsystem(edu.wpi.first.wpilibj2.command.Subsystem subsystem) {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.cancelAll();
    scheduler.removeDefaultCommand(subsystem);
    scheduler.registerSubsystem(subsystem);
  }
}
