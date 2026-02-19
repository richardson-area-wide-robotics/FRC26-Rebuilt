package frc.robot.common.components.hardware;

import frc.robot.common.swerve.RAWRSwerveModule;
import org.lasarobotics.drive.swerve.SwerveModule;
import org.lasarobotics.hardware.revrobotics.Spark;
import frc.robot.common.interfaces.IMU;
/**
 * Parameter bundle for creating SwerveHardware
 */
public record SwerveHardwareParams(
        IMU imu,

        Spark.ID leftFrontDriveId,
        Spark.ID leftFrontRotateId,

        Spark.ID rightFrontDriveId,
        Spark.ID rightFrontRotateId,

        Spark.ID leftRearDriveId,
        Spark.ID leftRearRotateId,

        Spark.ID rightRearDriveId,
        Spark.ID rightRearRotateId
) {

    /**
     * Initialize hardware devices as a real {@link SwerveHardware} type
     *
     * @return A Hardware object containing all necessary devices for this subsystem
     */
    public SwerveHardware initializeHardware() {

        RAWRSwerveModule lFrontModule = RAWRSwerveModule.createSwerve(
                this.leftFrontDriveId(),
                this.leftFrontRotateId(),
                SwerveModule.Location.LeftFront
        );

        RAWRSwerveModule rFrontModule = RAWRSwerveModule.createSwerve(
                this.rightFrontDriveId(),
                this.rightFrontRotateId(),
                SwerveModule.Location.RightFront
        );

        RAWRSwerveModule lRearModule = RAWRSwerveModule.createSwerve(
                this.leftRearDriveId(),
                this.leftRearRotateId(),
                SwerveModule.Location.LeftRear
        );

        RAWRSwerveModule rRearModule = RAWRSwerveModule.createSwerve(
                this.rightRearDriveId(),
                this.rightRearRotateId(),
                SwerveModule.Location.RightRear
        );

        return new SwerveHardware(this.imu(), lFrontModule, rFrontModule, lRearModule, rRearModule);
    }

}
