package frc.robot.common.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.components.hardware.swerve.MAXSwerveDrivetrain;
import frc.robot.CommonConstants.DriveConstants;
import frc.robot.common.interfaces.IRobotContainer;

public class SwerveDriveSubsystem extends SubsystemBase implements ModulePositionSupplier {

  private final MAXSwerveDrivetrain SWERVE_DRIVETRAIN = new MAXSwerveDrivetrain(
          // Front Left
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset,

          // Front Right
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset,

          // Back Left
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset,

          // Back Right
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset
  );

  private final IRobotContainer robotContainer;

  /**
   * Create the DriveSubsystem.
   * */
  public SwerveDriveSubsystem() {
    this.robotContainer = Robot.getRobotContainer();

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  @Override
  public void periodic() {
    //assumedPose.periodic();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    robotContainer.getPose2dSupplier().getPose().getRotation()
            )
                    : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
    );

    SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates,
            DriveConstants.kMaxSpeedMetersPerSecond
    );

    SWERVE_DRIVETRAIN.setStates(swerveModuleStates);
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    drive(
      chassisSpeeds.vxMetersPerSecond,
      chassisSpeeds.vyMetersPerSecond,
      chassisSpeeds.omegaRadiansPerSecond,
      false);
  }

  public Command driveCommand(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    return run(() -> drive(xSpeed, ySpeed, rot, fieldRelative));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    SWERVE_DRIVETRAIN.setStates(desiredStates);
  }




  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds();
  }

  public void configureAutoBuilder() {
    AutoBuilder.configure(
            ()-> robotContainer.getPose2dSupplier().getPose(), // Robot pose supplier
            (pose) -> robotContainer.getPose2dSupplier().resetPose(pose), // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(14.0, 0.0, 0.1), // Translation PID constants
                    new PIDConstants(2.1, 0.0, 0.2) // Rotation PID constants
            ),
            RobotUtils.getRobotConfig(), // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  @Override
  public SwerveModulePosition[] getSwerveModulePositions() {
    return SWERVE_DRIVETRAIN.getPositions();
  }
}
