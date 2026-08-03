// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.strubium.ssjprofiler.Profiler;
import com.strubium.ssjprofiler.ProfilerGlobal;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.common.LocalADStarAK;
import frc.robot.common.components.TeamUtils;
import frc.robot.common.components.dashboard.DashboardAutoUpdater;
import frc.robot.common.components.RobotContainerRegistry;
import frc.robot.common.components.RobotExceptionHandler;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.components.diagnostics.ExpectationMonitor;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.common.interfaces.IRobotContainer;
import org.littletonrobotics.junction.Logger;


/**
 * "Starting point" of the robot, nothing in here should need to be touched.
 *
 * This sets up the ExceptionHandler, PathPlanner, Logging, and then creates a IRobotContainer based off the team number
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private IRobotContainer robotContainer;

  public Robot() {
    super();
  }

  @Override
  public void robotInit() {
    // Logging must come up before anything else, so that any problem during the rest of
    // robotInit() is actually captured somewhere we can read back.
    startLogging();

    Profiler profiler = new Profiler("robot init");
    profiler.start();

    DriverStation.silenceJoystickConnectionWarning(CommonConstants.HIDConstants.SILENCE_NO_CONTROLLER_WARNING);

    Thread.setDefaultUncaughtExceptionHandler(new RobotExceptionHandler());


    // Set pathfinding algorithm to be AdvantageKit compatible
    Pathfinding.setPathfinder(new LocalADStarAK());

    System.out.println("Starting with team: " + TeamUtils.getTeamNumber());
    RobotUtils.loadRobotConfig();
    robotContainer = RobotContainerRegistry.createContainerForTeam(TeamUtils.getTeamNumber());

    robotContainer.robotInit();
    profiler.end();
  }

  /**
   * Brings AdvantageKit up.
   *
   * <p>Without this, every {@code Logger.recordOutput(...)} call in the codebase goes
   * nowhere: no log file is written, nothing is published to NetworkTables, and
   * AdvantageScope shows an empty session.
   *
   * <p>On a real robot we write a WPILOG (to a USB stick when one is mounted) and also
   * publish live to NetworkTables. In simulation we publish to NetworkTables only, so sim
   * runs don't litter the working tree with log files.
   */
  private void startLogging() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("GitDirty", switch (BuildConstants.DIRTY) {
      case 0 -> "All changes committed";
      case 1 -> "Uncommitted changes";
      default -> "Unknown";
    });
    Logger.recordMetadata("TeamNumber", Integer.toString(TeamUtils.getTeamNumber()));

    if (RobotBase.isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
    }
    Logger.addDataReceiver(new NT4Publisher());

    Logger.start();
  }


  @Override
  public void robotPeriodic() {
    logController("HID/Driver", CommonConstants.HIDConstants.DRIVER_CONTROLLER);
    logController("HID/Operator", CommonConstants.HIDConstants.OPERATOR_CONTROLLER);

    DashboardAutoUpdater.updateAll();
    CommandScheduler.getInstance().run();

    // Runs in every mode, so field state and health checks stay live during autonomous
    // too — previously this work only happened in teleop.
    robotContainer.robotPeriodic();
    ExpectationMonitor.getInstance().update();
  }


  private void logController(String name, CommandXboxController controller) {
    Logger.recordOutput(name + "/LeftX", controller.getLeftX());
    Logger.recordOutput(name + "/LeftY", controller.getLeftY());
    Logger.recordOutput(name + "/RightX", controller.getRightX());
    Logger.recordOutput(name + "/RightY", controller.getRightY());
    Logger.recordOutput(name + "/AButton", controller.a().getAsBoolean());
    Logger.recordOutput(name + "/BButton", controller.b().getAsBoolean());
    Logger.recordOutput(name + "/Connected", controller.isConnected());
  }

  @Override
  public void disabledPeriodic() {
    robotContainer.disabledPeriodic();
  }

  @Override
  public void autonomousInit() {
    Profiler profiler = new Profiler("auton init");
    profiler.start();
    robotContainer.autonomousInit();

    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      Logger.recordOutput("Auto/AutonomousCommand", autonomousCommand.getName());

      CommandScheduler.getInstance().schedule(autonomousCommand);
    } else {
      Logger.recordOutput("Auto/AutonomousCommand", "<none selected>");
    }
    profiler.end();
  }

  @Override
  public void autonomousPeriodic() {
    robotContainer.autonomousPeriodic();

  }

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    robotContainer.teleopPeriodic();

  }

  @Override
  public void simulationPeriodic() {
    robotContainer.simulationPeriodic();
  }

  /**
   * Entering Test mode runs the robot's on-blocks self-test.
   *
   * <p>Put the robot on blocks with the wheels clear, then select Test on the driver station.
   * Results print to the console and land in AdvantageKit under the suite's
   * {@code Validation} subtable.
   */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    Command validation = robotContainer.getValidationCommand();
    if (validation != null) {
      System.out.println("Test mode: running on-robot validation suite");
      CommandScheduler.getInstance().schedule(validation);
    } else {
      System.out.println("Test mode: this robot has no validation suite");
    }
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void endCompetition() {
    System.out.println("Simulation shutting down");
    ProfilerGlobal.exportToFile("profile_log.txt");
  }


}
