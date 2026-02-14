// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pearce;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CommonConstants;
import frc.robot.CommonConstants.HIDConstants;
import frc.robot.common.annotations.Robot;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.components.hardware.SwerveHardwareParams;
import frc.robot.common.gyro.RAWRNavX2;
import frc.robot.common.interfaces.IRobotContainer;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.pearce.components.HubStatus;
import frc.robot.pearce.subsystems.ProtoClimber;
import frc.robot.pearce.subsystems.ProtoFeeder;
import frc.robot.pearce.subsystems.ProtoIntake;
import frc.robot.pearce.subsystems.ProtoShooter;
import frc.robot.pearce.subsystems.smart.TeleopAssistSubsystem;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;
import org.lasarobotics.utils.PIDConstants;
import org.littletonrobotics.junction.Logger;

import static org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity.FIELD_CENTRIC;


@NoArgsConstructor(access = AccessLevel.PRIVATE)
@Robot(team = 1745)
public class PearceContainer implements IRobotContainer {


  public static final ProtoShooter PROTO_SHOOTER = new ProtoShooter(10, 11);
  public static final ProtoFeeder PROTO_FEEDER = new ProtoFeeder(18);

  //public static final ProtoClimber PROTO_CLIMBER = new ProtoClimber(15);
  public static final SwerveDriveSubsystem DRIVE_SUBSYSTEM = new SwerveDriveSubsystem(
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
          PIDConstants.of(4.0, 0.0, 0.05, 0.0, 0.0),
          FIELD_CENTRIC,
      CommonConstants.DriveConstants.BASIC_DRIVE_THROTTLE_INPUT_CURVE,
      CommonConstants.DriveConstants.BASIC_DRIVE_TURN_INPUT_CURVE,
      Angle.ofRelativeUnits(CommonConstants.DriveConstants.DRIVE_TURN_SCALAR, Units.Degree),
      Dimensionless.ofRelativeUnits(CommonConstants.HIDConstants.CONTROLLER_DEADBAND, Units.Value),
      Time.ofRelativeUnits(CommonConstants.DriveConstants.DRIVE_LOOKAHEAD, Units.Second));

  public static final TeleopAssistSubsystem TELEOP_ASSIST = new TeleopAssistSubsystem(DRIVE_SUBSYSTEM);

  private static SendableChooser<Command> automodeChooser;

  public static IRobotContainer createContainer(){
        // Set drive command
        // LeftY is the xRequest and LeftX is the yRequest for some reason
        DRIVE_SUBSYSTEM.setDefaultCommand(
        DRIVE_SUBSYSTEM.driveCommand(
            HIDConstants.DRIVER_CONTROLLER::getLeftY,
            HIDConstants.DRIVER_CONTROLLER::getLeftX,
            HIDConstants.DRIVER_CONTROLLER::getRightX));

      // Bind buttons and triggers
      configureBindings();
  
      // Set up the auto builder
      DRIVE_SUBSYSTEM.configureAutoBuilder();

      // Set up the auto chooser
      automodeChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData(CommonConstants.SmartDashboardConstants.SMARTDASHBOARD_AUTO_MODE, automodeChooser);

      return new PearceContainer();
  }

  private static void configureBindings() {
    // Driver Start - toggle traction control
    //RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.start(), DRIVE_SUBSYSTEM.toggleTractionControlCommand(), Commands.none());

    // Driver Left Stick Button - Reset pose
    //RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.leftStick(), DRIVE_SUBSYSTEM.resetPoseCommand(Pose2d::new), Commands.none());

    // Driver Right Stick Button - Reset heading
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.rightStick(), Commands.runOnce(DRIVE_SUBSYSTEM.DRIVETRAIN_HARDWARE.gyro()::reset, DRIVE_SUBSYSTEM), Commands.none());

    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.a(), Commands.runOnce(PROTO_SHOOTER::runShooter, PROTO_SHOOTER), Commands.runOnce(PROTO_SHOOTER::stopShooter));

    RobotUtils.bindControl(
            HIDConstants.DRIVER_CONTROLLER.leftBumper(),
            Commands.runOnce(TELEOP_ASSIST::toggle), Commands.none()
    );

    RobotUtils.bindControl(
            HIDConstants.DRIVER_CONTROLLER.povUp(),
            Commands.runOnce(TELEOP_ASSIST::disable), Commands.none()
    );

    RobotUtils.bindControl(
            HIDConstants.DRIVER_CONTROLLER.b(),
            Commands.runOnce(PROTO_FEEDER::load),
            Commands.runOnce(PROTO_FEEDER::stopLoad));


    //RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.leftTrigger(), Commands.runOnce(PROTO_CLIMBER::runClimber, PROTO_CLIMBER), Commands.runOnce(PROTO_CLIMBER::stopClimber));

      //RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.rightTrigger(), Commands.runOnce(PROTO_CLIMBER::unRunClimber, PROTO_CLIMBER), Commands.runOnce(PROTO_CLIMBER::stopClimber));

//    RobotUtils.bindControl(
//      HIDConstants.DRIVER_CONTROLLER.b(),
//      Commands.runOnce(() -> {
//          Command coolPathCommand = AutoBuilder.buildAuto("coolpath.auto");
//          if (coolPathCommand != null) {
//              coolPathCommand.schedule();
//         }
//      }),
//      Commands.none()
//    );
  }


  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousPeriodic() {
  }
  
  @Override
  public void teleopPeriodic() {
    HubStatus.HubState[] statuses = HubStatus.getBothHubStatuses(DriverStation.getMatchTime());
    Logger.recordOutput("/Status/Red", statuses[0]);
    Logger.recordOutput("/Status/Blue", statuses[1]);
  }

  /**
   * Get currently selected autonomous command
   * 
   * @return Autonomous command
   */
  @Override
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
