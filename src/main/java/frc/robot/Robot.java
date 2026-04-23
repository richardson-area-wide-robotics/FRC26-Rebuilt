// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.strubium.ssjprofiler.Profiler;
import com.strubium.ssjprofiler.ProfilerGlobal;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.common.LocalADStarAK;
import frc.robot.common.components.TeamUtils;
import frc.robot.common.components.dashboard.DashboardAutoUpdater;
import frc.robot.common.components.RobotContainerRegistry;
import frc.robot.common.components.RobotExceptionHandler;
import frc.robot.common.components.RobotUtils;
import org.littletonrobotics.junction.LoggedRobot;
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


  @Override
  public void robotPeriodic() {
    logController("HID/Driver", CommonConstants.HIDConstants.DRIVER_CONTROLLER);
    logController("HID/Operator", CommonConstants.HIDConstants.OPERATOR_CONTROLLER);

    DashboardAutoUpdater.updateAll();
    //CANDiagnostics.getInstance().checkHealth();
    CommandScheduler.getInstance().run();
  }


  private void logController(String name, CommandXboxController controller) {
    Logger.recordOutput(name + "/LeftX", controller.getLeftX());
    Logger.recordOutput(name + "/LeftY", controller.getLeftY());
    Logger.recordOutput(name + "/RightX", controller.getRightX());
    Logger.recordOutput(name + "/RightY", controller.getRightY());
    Logger.recordOutput(name + "/AButton", controller.a().getAsBoolean());
    Logger.recordOutput(name + "/BButton", controller.b().getAsBoolean());
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
    //PearceContainer.DRIVE_SUBSYSTEM.DRIVETRAIN_HARDWARE.navx.reset();
  }

  @Override
  public void teleopPeriodic() {
    robotContainer.teleopPeriodic();

  }

  @Override
  public void simulationPeriodic() {
    robotContainer.simulationPeriodic();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
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
