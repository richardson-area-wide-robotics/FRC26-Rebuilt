package frc.robot.common.components.hardware;

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
) {}
