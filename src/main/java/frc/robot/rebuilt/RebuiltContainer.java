// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CommonConstants;
import frc.robot.CommonConstants.HIDConstants;
import frc.robot.common.annotations.Robot;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.components.hardware.SwerveHardwareParams;
import frc.robot.common.gyro.RAWRNavX2;
import frc.robot.common.interfaces.IRobotContainer;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.rebuilt.components.HubStatus;
import frc.robot.rebuilt.components.RobotSector;
import frc.robot.rebuilt.subsystems.Feeder;
import frc.robot.rebuilt.subsystems.Intake;
import frc.robot.rebuilt.subsystems.Shooter;
import frc.robot.rebuilt.subsystems.Shooter.ShooterPosition;
import frc.robot.rebuilt.subsystems.smart.RobotSectorEvaluator;
import frc.robot.rebuilt.subsystems.smart.ScoringLocationLookup;
import frc.robot.rebuilt.subsystems.smart.SmartSequentialCommandSequencer;
import frc.robot.rebuilt.components.SmartSequentialCommandContainer;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;
import org.lasarobotics.utils.PIDConstants;
import org.littletonrobotics.junction.Logger;

import static org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity.FIELD_CENTRIC;


@NoArgsConstructor(access = AccessLevel.PRIVATE)
@Robot(team = 1745)
public class RebuiltContainer implements IRobotContainer {


  public static Shooter SHOOTER;
  public static Feeder FEEDER;
  public static Intake INTAKE;
  public static SwerveDriveSubsystem DRIVE_SUBSYSTEM;

  public static RobotSectorEvaluator SECTOR_EVALUATOR;
  public static SmartSequentialCommandSequencer COMMAND_SEQUENCER;
  public static SequentialCommandGroup sequencedCommand;

  private static SendableChooser<Command> automodeChooser;
  public static boolean weWonAuton = false;
  public static boolean hubOn = true;
  public static boolean hubBlinking = false;

  public static IRobotContainer createContainer(){
    SHOOTER = new Shooter(RebuiltConstants.HardwareConstants.SHOOTER_LEADER_ID, RebuiltConstants.HardwareConstants.SHOOTER_FOLLOWER_ID);
    FEEDER = new Feeder(RebuiltConstants.HardwareConstants.FEEDER_MOTOR_ID, RebuiltConstants.HardwareConstants.SPINDEXER_MOTOR_ID);
    INTAKE = new Intake(RebuiltConstants.HardwareConstants.INTAKE_MOTOR_1_ID, RebuiltConstants.HardwareConstants.INTAKE_MOTOR_2_ID, RebuiltConstants.HardwareConstants.INTAKE_DEPLOY_MOTOR_ID);
    DRIVE_SUBSYSTEM = new SwerveDriveSubsystem(
                new SwerveHardwareParams(
                        new RAWRNavX2(CommonConstants.DriveHardwareConstants.NAVX_NAME),

                        CommonConstants.DriveHardwareConstants.LEFT_FRONT_DRIVE_MOTOR_ID,
                        CommonConstants.DriveHardwareConstants.LEFT_FRONT_ROTATE_MOTOR_ID,

                        CommonConstants.DriveHardwareConstants.RIGHT_FRONT_DRIVE_MOTOR_ID,
                        CommonConstants.DriveHardwareConstants.RIGHT_FRONT_ROTATE_MOTOR_ID,
                        CommonConstants.DriveHardwareConstants.LEFT_REAR_DRIVE_MOTOR_ID,
                        CommonConstants.DriveHardwareConstants.LEFT_REAR_ROTATE_MOTOR_ID,

                        CommonConstants.DriveHardwareConstants.RIGHT_REAR_DRIVE_MOTOR_ID,
                        CommonConstants.DriveHardwareConstants.RIGHT_REAR_ROTATE_MOTOR_ID
                ),
          PIDConstants.of(14.0, 0.0, 0.1, 0.0, 0.0),
          FIELD_CENTRIC,
      CommonConstants.DriveConstants.BASIC_DRIVE_THROTTLE_INPUT_CURVE,
      CommonConstants.DriveConstants.BASIC_DRIVE_TURN_INPUT_CURVE,
      Angle.ofRelativeUnits(CommonConstants.DriveConstants.DRIVE_TURN_SCALAR, Units.Degree),
      Dimensionless.ofRelativeUnits(CommonConstants.HIDConstants.CONTROLLER_DEADBAND, Units.Value),
      Time.ofRelativeUnits(CommonConstants.DriveConstants.DRIVE_LOOKAHEAD, Units.Second));

    SECTOR_EVALUATOR = new RobotSectorEvaluator(DRIVE_SUBSYSTEM);
    COMMAND_SEQUENCER = new SmartSequentialCommandSequencer(SmartSequentialCommandContainer.goToRedHub);

        // Set drive command
        // LeftY is the xRequest and LeftX is the yRequest for some reason
        DRIVE_SUBSYSTEM.setDefaultCommand(
        DRIVE_SUBSYSTEM.driveCommand(
            HIDConstants.DRIVER_CONTROLLER::getLeftY,
            HIDConstants.DRIVER_CONTROLLER::getLeftX,
            HIDConstants.DRIVER_CONTROLLER::getRightX));

    // Set up the auto builder
    DRIVE_SUBSYSTEM.configureAutoBuilder();


    sequencedCommand = COMMAND_SEQUENCER.finalizeSequence();

      // Bind buttons and triggers
      configureBindings();
  
      // Register named commands
      registerNamedCommands();

      // Set up the auto chooser
      automodeChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData(CommonConstants.SmartDashboardConstants.SMARTDASHBOARD_AUTO_MODE, automodeChooser);

      SECTOR_EVALUATOR.createSector(RobotSector.BaseSector.BLUE, RobotSector.SectorType.TOWER,new Pose2d(1.,1.,new Rotation2d()),1,1);
      SECTOR_EVALUATOR.createSector(RobotSector.BaseSector.BLUE, RobotSector.SectorType.TOWER,new Pose2d(3.,1.,new Rotation2d()),1,1);
      SECTOR_EVALUATOR.createSector(RobotSector.BaseSector.BLUE, RobotSector.SectorType.TOWER,new Pose2d(1.,3.,new Rotation2d()),1,1);
      SECTOR_EVALUATOR.createSector(RobotSector.BaseSector.BLUE, RobotSector.SectorType.TOWER,new Pose2d(3.,3.,new Rotation2d()),1,1);

      // Set up scoring location lookup
      ScoringLocationLookup.buildScoringLocations();

    return new RebuiltContainer();
  }


  private static void configureBindings() {
    // Operator Start - toggle traction control
    RobotUtils.bindControl(HIDConstants.OPERATOR_CONTROLLER.start(), DRIVE_SUBSYSTEM.toggleTractionControlCommand(), Commands.none());

    // Operator Left Stick Button - Reset pose
    RobotUtils.bindControl(HIDConstants.OPERATOR_CONTROLLER.leftStick(), DRIVE_SUBSYSTEM.resetPoseCommand(Pose2d::new), Commands.none());

    // Operator A Button - Lock Wheels
    RobotUtils.bindControl(HIDConstants.OPERATOR_CONTROLLER.a(), DRIVE_SUBSYSTEM.lockCommand(), Commands.none());

    // Operator POV Up - Lower Shooter RPM by 10
    RobotUtils.bindControl(HIDConstants.OPERATOR_CONTROLLER.povDown(), Commands.runOnce(()-> SHOOTER.lowerOperatorModifer(10)) , Commands.none());

    // Operator POV Up - Raise Shooter RPM by 10
    RobotUtils.bindControl(HIDConstants.OPERATOR_CONTROLLER.povUp(), Commands.runOnce(()-> SHOOTER.raiseOperatorModifer(10)) , Commands.none());

    // Driver Right Stick Button - Reset heading
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.rightStick(), Commands.runOnce(DRIVE_SUBSYSTEM.DRIVETRAIN_HARDWARE.gyro()::reset, DRIVE_SUBSYSTEM), Commands.none());

    //Driver DPad Up - Deploy intake
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.povUp(),
      INTAKE.deploy(),
      Commands.runOnce(INTAKE::stopDeploy, INTAKE));

    //RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.povUp(),
    //  Commands.runOnce(INTAKE::manualDeploy, INTAKE),
    //  Commands.runOnce(INTAKE::stopDeploy, INTAKE));

    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.povDown(),
    Commands.runOnce(INTAKE::manualReverseDeploy, INTAKE),
    Commands.runOnce(INTAKE::stopDeploy, INTAKE));

    // Driver A Button - Shoot from hub
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.a(),
            Commands.runOnce(()->SHOOTER.setCurrentShooterPosition(Shooter.ShooterPosition.HUB))
                    .alongWith(Commands.run(SHOOTER::runShooter, SHOOTER)),
            Commands.runOnce(SHOOTER::idleOrStop));

    // Driver B Button - Shoot from trench
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.b(),
            Commands.runOnce(()->SHOOTER.setCurrentShooterPosition(Shooter.ShooterPosition.TRENCH))
                    .alongWith(Commands.run(SHOOTER::runShooter, SHOOTER)),
            Commands.runOnce(SHOOTER::idleOrStop));

    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.x(),
            Commands.runOnce(()->SHOOTER.setCurrentShooterPosition(Shooter.ShooterPosition.TOWER))
                    .alongWith(Commands.run(SHOOTER::runShooter, SHOOTER)),
            Commands.runOnce(SHOOTER::idleOrStop));

    // Driver Y Button - Shoot from corner
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.y(),
            Commands.runOnce(()->SHOOTER.setCurrentShooterPosition(Shooter.ShooterPosition.CORNER))
                    .alongWith(Commands.run(SHOOTER::runShooter, SHOOTER)),
            Commands.runOnce(SHOOTER::idleOrStop));


    // Driver Right Bumper - Reverse Load
    RobotUtils.bindControl(
            HIDConstants.DRIVER_CONTROLLER.rightBumper(),
            Commands.runOnce(FEEDER::reverseLoad).alongWith(Commands.runOnce(FEEDER::reverseCycle)),
            Commands.runOnce(FEEDER::stopLoad).alongWith(Commands.runOnce(FEEDER::stopCycle)));

    // Driver Right Trigger - Load (fire balls if shooter is on)
    RobotUtils.bindControl(
            HIDConstants.DRIVER_CONTROLLER.rightTrigger(),
            Commands.runOnce(FEEDER::load)
              .alongWith(Commands.runOnce(FEEDER::cycle)
              .alongWith(Commands.runOnce(() -> INTAKE.manualReverseDeploy()))
              .alongWith(INTAKE.intakeCommand())),
            Commands.runOnce(FEEDER::stopLoad)
              .alongWith(Commands.runOnce(FEEDER::stopCycle)
              .alongWith(INTAKE.stopIntakeCommand())));

    // Driver Left Trigger - Intake
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.leftTrigger(),
      Commands.runOnce(INTAKE::intake),
      Commands.runOnce(INTAKE::stop));

    // Driver Left Bumper - Outtake
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.leftBumper(),
      Commands.runOnce(() -> INTAKE.outtake()),
      Commands.runOnce(() -> INTAKE.stop()));

    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.povLeft(),
            DRIVE_SUBSYSTEM.aimAtPointCommand(ScoringLocationLookup.findHub().getTranslation(), false, false),
            Commands.runOnce(DRIVE_SUBSYSTEM::stopCommand));
  }

  private static void registerNamedCommands() {
    NamedCommands.registerCommand("Set Shooter Hub", Commands.runOnce(() -> SHOOTER.setCurrentShooterPosition(Shooter.ShooterPosition.HUB)));
    NamedCommands.registerCommand("Set Shooter Trench", Commands.runOnce(() -> SHOOTER.setCurrentShooterPosition(Shooter.ShooterPosition.TRENCH)));
    NamedCommands.registerCommand("Set Shooter Corner", Commands.runOnce(() -> SHOOTER.setCurrentShooterPosition(Shooter.ShooterPosition.CORNER)));
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

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
  }
  
  @Override
  public void teleopPeriodic() {
    HubStatus.HubState[] statuses = HubStatus.getBothHubStatuses(DriverStation.getMatchTime());
    if(ScoringLocationLookup.team == null){
        ScoringLocationLookup.team = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }
    Logger.recordOutput("Assist/ShooterPosition",ScoringLocationLookup.findClosest(DRIVE_SUBSYSTEM.getPose()));
    Logger.recordOutput("Assist/HubPose", ScoringLocationLookup.findHub());

    Logger.recordOutput("Status/Red", statuses[0]);
    Logger.recordOutput("Status/Blue", statuses[1]);

    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      hubOn = statuses[0] == HubStatus.HubState.ACTIVE;
    }
    else{
      hubOn = statuses[1] == HubStatus.HubState.ACTIVE;
    }
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      hubBlinking = statuses[0] == HubStatus.HubState.BLINKING;
    }
    else{
      hubBlinking = statuses[1] == HubStatus.HubState.BLINKING;
    }

    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        weWonAuton = "r".equals(DriverStation.getGameSpecificMessage());
    }
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
      weWonAuton = "b".equals(DriverStation.getGameSpecificMessage());
    }

    Logger.recordOutput("AUTONWINNERS", weWonAuton);

    Logger.recordOutput("HUB", hubOn);
    Logger.recordOutput("HUBWARNING", hubBlinking);
  }

  /**
   * Get currently selected autonomous command
   * 
   * @return Autonomous command
   */
  @Override
  public Command getAutonomousCommand() {
    return automodeChooser.getSelected();
  }
}
