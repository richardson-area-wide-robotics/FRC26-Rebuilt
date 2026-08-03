// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.strubium.ssjprofiler.Profiler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CommonConstants;
import frc.robot.CommonConstants.HIDConstants;
import frc.robot.common.annotations.Robot;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.components.diagnostics.ExpectationMonitor;
import frc.robot.common.interfaces.IRobotContainer;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.rebuilt.RebuiltConstants.CanIds;
import frc.robot.rebuilt.RebuiltConstants.IntakeConstants;
import frc.robot.rebuilt.RebuiltConstants.ShooterConstants;
import frc.robot.rebuilt.components.FieldState;
import frc.robot.rebuilt.components.RobotSector;
import frc.robot.rebuilt.subsystems.Feeder;
import frc.robot.rebuilt.subsystems.Intake;
import frc.robot.rebuilt.subsystems.Shooter;
import frc.robot.rebuilt.subsystems.Shooter.ShooterPosition;
import frc.robot.rebuilt.subsystems.smart.RobotSectorEvaluator;
import frc.robot.rebuilt.subsystems.smart.ScoringLocationLookup;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;
import org.littletonrobotics.junction.Logger;


@NoArgsConstructor(access = AccessLevel.PRIVATE)
@Robot(team = 1745)
public class RebuiltContainer implements IRobotContainer {

  /** Owns alliance and hub state. Refreshed every loop, in every mode. */
  public static final FieldState FIELD_STATE = new FieldState();

  public static final Shooter SHOOTER =
      new Shooter(CanIds.SHOOTER_LEADER, CanIds.SHOOTER_FOLLOWER, FIELD_STATE::isHubActive);
  public static final Feeder FEEDER = new Feeder(CanIds.FEEDER, CanIds.SPINDEXER);
  public static final Intake INTAKE =
      new Intake(CanIds.INTAKE_ROLLER_1, CanIds.INTAKE_ROLLER_2, CanIds.INTAKE_DEPLOY);
  public static final SwerveDriveSubsystem DRIVE_SUBSYSTEM = new SwerveDriveSubsystem();

  public static final RobotSectorEvaluator SECTOR_EVALUATOR = new RobotSectorEvaluator(DRIVE_SUBSYSTEM);

  private static SendableChooser<Command> automodeChooser;

  public static IRobotContainer createContainer() {
        // Set drive command.
        //
        // These MUST be method references, not evaluated values: passing
        // controller.getLeftX() directly captured a single reading taken at class-load
        // time, so the drivetrain was commanded to a frozen zero for the whole match.
        //
        // LeftY drives the x request and LeftX the y request because the field frame has
        // +x forward and +y left, while the stick reports +y backward and +x right — hence
        // the negations.
        DRIVE_SUBSYSTEM.setDefaultCommand(
          DRIVE_SUBSYSTEM.driveCommand(
            () -> -HIDConstants.DRIVER_CONTROLLER.getLeftY(),
            () -> -HIDConstants.DRIVER_CONTROLLER.getLeftX(),
            () -> -HIDConstants.DRIVER_CONTROLLER.getRightX(),
            true)
        );

    // Set up the auto builder
    DRIVE_SUBSYSTEM.configureAutoBuilder();

      // Bind buttons and triggers
      configureBindings();

      // Register named commands
      registerNamedCommands();

      // Set up the auto chooser
      automodeChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData(CommonConstants.SmartDashboardConstants.SMARTDASHBOARD_AUTO_MODE, automodeChooser);

      // TODO: these four sectors are placeholder test data — all BLUE/TOWER in a 1x1 grid
      // near the origin. Replace with real field geometry before competition.
      SECTOR_EVALUATOR.createSector(RobotSector.BaseSector.BLUE, RobotSector.SectorType.TOWER, new Pose2d(1., 1., new Rotation2d()), 1, 1);
      SECTOR_EVALUATOR.createSector(RobotSector.BaseSector.BLUE, RobotSector.SectorType.TOWER, new Pose2d(3., 1., new Rotation2d()), 1, 1);
      SECTOR_EVALUATOR.createSector(RobotSector.BaseSector.BLUE, RobotSector.SectorType.TOWER, new Pose2d(1., 3., new Rotation2d()), 1, 1);
      SECTOR_EVALUATOR.createSector(RobotSector.BaseSector.BLUE, RobotSector.SectorType.TOWER, new Pose2d(3., 3., new Rotation2d()), 1, 1);

      // Set up scoring location lookup
      ScoringLocationLookup.buildScoringLocations();

      registerExpectations();

    return new RebuiltContainer();
  }

  /**
   * Registers the invariants that say "this robot is working".
   *
   * <p>These are observed every loop and logged under {@code Expectations/}. They are the
   * automated form of the checks a driver would otherwise make by feel, and they exist
   * because the most serious bug in this codebase — a drivetrain that ignored the sticks —
   * was invisible without them.
   */
  private static void registerExpectations() {
    ExpectationMonitor monitor = ExpectationMonitor.getInstance();

    monitor.register(
        "DriveRespondsToStick",
        "When the driver commands motion, at least one module is commanded to move",
        () -> {
          double commanded = Math.abs(SwerveDriveSubsystem.applyDeadband(
                  HIDConstants.DRIVER_CONTROLLER.getLeftY()))
              + Math.abs(SwerveDriveSubsystem.applyDeadband(
                  HIDConstants.DRIVER_CONTROLLER.getLeftX()))
              + Math.abs(SwerveDriveSubsystem.applyDeadband(
                  HIDConstants.DRIVER_CONTROLLER.getRightX()));
          if (commanded <= 0) {
            return true; // Nothing asked for, nothing to check.
          }
          for (var state : DRIVE_SUBSYSTEM.getDesiredModuleStates()) {
            if (Math.abs(state.speedMetersPerSecond) > 1e-3) {
              return true;
            }
          }
          return false;
        },
        5);

    monitor.register(
        "HeadingFinite",
        "The gyro is reporting a usable heading",
        () -> Double.isFinite(DRIVE_SUBSYSTEM.getHeading()));

    monitor.register(
        "PoseFinite",
        "The pose estimate has not diverged to NaN or infinity",
        () -> {
          Pose2d pose = DRIVE_SUBSYSTEM.getPose();
          return Double.isFinite(pose.getX())
              && Double.isFinite(pose.getY())
              && Double.isFinite(pose.getRotation().getRadians());
        });

    monitor.register(
        "ShooterReachesSetpoint",
        "Once commanded, the flywheel arrives at its target RPM and stays there",
        () -> !SHOOTER.isRunning() || SHOOTER.isAtTargetRPM(),
        // Generous: spin-up from rest to 4500 RPM legitimately takes a few seconds.
        150);

    monitor.register(
        "ShooterRespectsHubInterlock",
        "The flywheel is never commanded while our hub is closed",
        () -> SHOOTER.isHubActive() || !SHOOTER.isRunning());

    monitor.register(
        "IntakeWithinSoftLimits",
        "The deploy arm stays inside its configured travel",
        () -> {
          double position = INTAKE.getDeployPosition();
          return position >= IntakeConstants.DEPLOY_REVERSE_SOFT_LIMIT - 1.0
              && position <= IntakeConstants.DEPLOY_FORWARD_SOFT_LIMIT + 1.0;
        });
  }


  private static void configureBindings() {
    Profiler bindingProfiler = new Profiler("bindings");
    bindingProfiler.start();

    // Operator POV Down - Lower Shooter RPM trim
    RobotUtils.bindControl(HIDConstants.OPERATOR_CONTROLLER.povDown(),
      Commands.runOnce(() -> SHOOTER.lowerOperatorModifer(ShooterConstants.OPERATOR_TRIM_STEP_RPM)),
      Commands.none());

    // Operator POV Up - Raise Shooter RPM trim
    RobotUtils.bindControl(HIDConstants.OPERATOR_CONTROLLER.povUp(),
      Commands.runOnce(() -> SHOOTER.raiseOperatorModifer(ShooterConstants.OPERATOR_TRIM_STEP_RPM)),
      Commands.none());

    // Driver Right Stick Button - Reset heading
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.rightStick(), Commands.runOnce(DRIVE_SUBSYSTEM::zeroHeading, DRIVE_SUBSYSTEM), Commands.none());

    //Driver DPad Up - Deploy intake
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.povUp(),
      INTAKE.deploy(),
      Commands.runOnce(INTAKE::stopDeploy, INTAKE));

    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.povDown(),
    Commands.runOnce(INTAKE::manualReverseDeploy, INTAKE),
    Commands.runOnce(INTAKE::stopDeploy, INTAKE));

    // Driver A Button - Shoot from hub
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.a(),
            shootFrom(ShooterPosition.HUB),
            Commands.runOnce(SHOOTER::idleOrStop));

    // Driver B Button - Shoot from trench
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.b(),
            shootFrom(ShooterPosition.TRENCH),
            Commands.runOnce(SHOOTER::idleOrStop));

    // Driver X Button - Shoot from tower
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.x(),
            shootFrom(ShooterPosition.TOWER),
            Commands.runOnce(SHOOTER::idleOrStop));

    // Driver Y Button - Shoot from corner
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.y(),
            shootFrom(ShooterPosition.CORNER),
            Commands.runOnce(SHOOTER::idleOrStop));


    // Driver Right Bumper - Reverse Load
    RobotUtils.bindControl(
            HIDConstants.DRIVER_CONTROLLER.rightBumper(),
            Commands.runOnce(() -> {
              FEEDER.reverseLoad();
              FEEDER.reverseCycle();
            }, FEEDER),
            Commands.runOnce(() -> {
              FEEDER.stopLoad();
              FEEDER.holdCycle();
            }, FEEDER));

    // Driver Right Trigger - Load (fire balls if shooter is on)
    //
    // Both intake actions must live in a SINGLE command: a parallel composition cannot
    // contain two commands that require the same subsystem, and now that these commands
    // declare INTAKE as a requirement, splitting them throws at construction time.
    RobotUtils.bindControl(
            HIDConstants.DRIVER_CONTROLLER.rightTrigger(),
            Commands.runOnce(() -> {
              FEEDER.load();
              FEEDER.cycle();
            }, FEEDER).alongWith(
              Commands.runOnce(() -> {
                INTAKE.manualReverseDeploy();
                INTAKE.intake();
              }, INTAKE)),
            Commands.runOnce(() -> {
              FEEDER.stopLoad();
              FEEDER.holdCycle();
            }, FEEDER).alongWith(
              Commands.runOnce(() -> {
                INTAKE.stopDeploy();
                INTAKE.holdRollers();
              }, INTAKE)));

    // Driver Left Trigger - Intake
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.leftTrigger(),
      Commands.runOnce(INTAKE::intake, INTAKE),
      Commands.runOnce(INTAKE::holdRollers, INTAKE));

    // Driver Left Bumper - Outtake
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.leftBumper(),
      Commands.runOnce(INTAKE::outtake, INTAKE),
      Commands.runOnce(INTAKE::stopRollers, INTAKE));

    bindingProfiler.end();
  }

  /**
   * Builds the command that selects a shooter preset and holds the flywheel there.
   *
   * @param position Preset to select.
   * @return A command suitable for {@code whileTrue}.
   */
  private static Command shootFrom(ShooterPosition position) {
    return Commands.runOnce(() -> SHOOTER.setCurrentShooterPosition(position))
        .alongWith(Commands.run(SHOOTER::runShooter, SHOOTER));
  }

  private static void registerNamedCommands() {
    NamedCommands.registerCommand("Set Shooter Hub", Commands.runOnce(() -> SHOOTER.setCurrentShooterPosition(ShooterPosition.HUB)));
    NamedCommands.registerCommand("Set Shooter Trench", Commands.runOnce(() -> SHOOTER.setCurrentShooterPosition(ShooterPosition.TRENCH)));
    NamedCommands.registerCommand("Set Shooter Corner", Commands.runOnce(() -> SHOOTER.setCurrentShooterPosition(ShooterPosition.CORNER)));
    NamedCommands.registerCommand("Set Shooter Tower", Commands.runOnce(() -> SHOOTER.setCurrentShooterPosition(ShooterPosition.TOWER)));
  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void robotInit() {
  }

  /**
   * Refreshes field state and driver-assist telemetry in every mode.
   *
   * <p>All of this used to live in {@link #teleopPeriodic()}, which meant the hub interlock
   * was stale throughout autonomous and every read of the alliance risked throwing.
   */
  @Override
  public void robotPeriodic() {
    FIELD_STATE.update();

    ScoringLocationLookup.setRedAlliance(FIELD_STATE.isAllianceRed());

    Logger.recordOutput("Assist/ShooterPosition",
        ScoringLocationLookup.findClosest(DRIVE_SUBSYSTEM.getPose()));
    Logger.recordOutput("Assist/HubPose", ScoringLocationLookup.findHub());
    Logger.recordOutput("Field/GameDataNamesUs", FIELD_STATE.gameDataNamesUs());
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

  /**
   * Get currently selected autonomous command
   *
   * @return Autonomous command
   */
  @Override
  public Command getAutonomousCommand() {
    return automodeChooser == null ? null : automodeChooser.getSelected();
  }

  /**
   * The on-blocks self-test. Enter Test mode on the driver station to run it.
   *
   * @return the validation command for this robot.
   */
  @Override
  public Command getValidationCommand() {
    return RebuiltValidation.build(DRIVE_SUBSYSTEM, SHOOTER, INTAKE, FEEDER);
  }
}
