package frc.robot.common.swerve;

import java.util.Map;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;

import org.lasarobotics.drive.swerve.DriveWheel;
import org.lasarobotics.drive.swerve.SwerveModule;
import org.lasarobotics.drive.swerve.parent.REVSwerveModule;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.utils.FFConstants;
import org.lasarobotics.utils.PIDConstants;

import frc.robot.CommonConstants;
import frc.robot.common.components.RobotUtils;

public class RAWRSwerveModule extends REVSwerveModule {

    public static final Map<SwerveModule.Location, Angle> ZERO_OFFSET = Map.ofEntries(
            Map.entry(SwerveModule.Location.LeftFront, Units.Radians.of(Math.PI / 2)),
            Map.entry(SwerveModule.Location.RightFront, Units.Radians.zero()),
            Map.entry(SwerveModule.Location.LeftRear, Units.Radians.of(Math.PI)),
            Map.entry(SwerveModule.Location.RightRear, Units.Radians.of(Math.PI / 2))
    );

    /**
     * Factory method used by drivetrain
     */
    public static RAWRSwerveModule createSwerve(
            Spark.ID driveMotor,
            Spark.ID rotateMotor,
            SwerveModule.Location location
    ) {

        REVSwerveModule.Hardware hardware =
                REVSwerveModule.initializeHardware(
                        driveMotor,
                        rotateMotor,
                        Spark.MotorKind.NEO_VORTEX,
                        Spark.MotorKind.NEO_550
                );

        return new RAWRSwerveModule(hardware, location);
    }

    private RAWRSwerveModule(
            REVSwerveModule.Hardware hardware,
            SwerveModule.Location location
    ) {

        super(
                hardware,
                location,

                SwerveModule.MountOrientation.STANDARD,
                SwerveModule.MountOrientation.INVERTED,

                CommonConstants.SwerveConstants.GEAR_RATIO,

                DriveWheel.create(
                        Distance.ofRelativeUnits(75, Units.Millimeter),
                        Dimensionless.ofBaseUnits(1.6, Units.Value),
                        Dimensionless.ofBaseUnits(1.3, Units.Value)
                ),

                ZERO_OFFSET.get(location),

                // Drive PID
                PIDConstants.of(0.18, 0, 0.174, 0, 0),

                // Drive FF
                FFConstants.of(0, 0, 0, 0),

                // Rotate PID
                PIDConstants.of(2.1, 0, 0.2, 0, 0),

                // Rotate FF
                FFConstants.of(0, 0, 0, 0),

                Dimensionless.ofBaseUnits(
                        CommonConstants.DriveConstants.DRIVE_SLIP_RATIO,
                        Units.Value
                ),

                Mass.ofRelativeUnits(
                        RobotUtils.getRobotConfig().massKG,
                        Units.Kilograms
                ),

                Distance.ofRelativeUnits(23, Units.Inches),
                Distance.ofRelativeUnits(24.5, Units.Inches),

                Time.ofBaseUnits(
                        CommonConstants.DriveConstants.AUTO_LOCK_TIME,
                        Units.Second
                ),

                Units.Amps.of(
                        CommonConstants.SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT
                )
        );
    }
}