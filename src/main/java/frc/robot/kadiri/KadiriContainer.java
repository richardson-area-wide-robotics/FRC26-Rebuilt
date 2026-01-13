package frc.robot.kadiri;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommonConstants;
import frc.robot.common.annotations.Robot;
import frc.robot.common.components.hardware.SwerveHardwareParams;
import frc.robot.common.gyro.RAWRNavX2;
import frc.robot.common.interfaces.IRobotContainer;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;
import org.lasarobotics.utils.PIDConstants;

import static org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity.FIELD_CENTRIC;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
@Robot(team = 6767)

public class KadiriContainer implements IRobotContainer {


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

    public static IRobotContainer createContainer(){
        DRIVE_SUBSYSTEM.setDefaultCommand(
                DRIVE_SUBSYSTEM.driveCommand(
                        CommonConstants.HIDConstants.DRIVER_CONTROLLER::getLeftY,
                        CommonConstants.HIDConstants.DRIVER_CONTROLLER::getLeftX,
                        CommonConstants.HIDConstants.DRIVER_CONTROLLER::getRightX));
        return new KadiriContainer();
    }


    @Override
    public Command getAutonomousCommand() {
        return null;
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

    }
}
