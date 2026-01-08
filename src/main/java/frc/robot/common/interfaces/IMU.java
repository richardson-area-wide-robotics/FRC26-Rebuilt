package frc.robot.common.interfaces;

import org.lasarobotics.drive.swerve.AdvancedSwerveKinematics.ControlCentricity;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * General-purpose IMU interface
 */
public interface IMU extends AutoCloseable {
    /** Periodically update the IMU sensor inputs */
    void updateInputs();

    /** {@return Update frequency of the sensor} */
    Frequency getUpdateRate();

    /** Reset gyro/IMU readings */
    void reset();

    /** {@return Whether sensor is currently connected} */
    boolean isConnected();

    /** {@return Whether sensor is currently calibrating} */
    boolean isCalibrating();

    /** {@return Roll angle} */
    Angle getRoll();

    /** {@return Pitch angle} */
    Angle getPitch();

    /** {@return Yaw angle} */
    Angle getYaw();

    /** {@return Yaw rate} */
    AngularVelocity getYawRate();

    /** {@return Rotation as Rotation2d} */
    Rotation2d getRotation2d();

    /** {@return X-axis velocity} */
    LinearVelocity getVelocityX();

    /** {@return Y-axis velocity} */
    LinearVelocity getVelocityY();

    /** Update simulated device values */
    void updateSim(Rotation2d orientation, ChassisSpeeds desiredSpeeds, ControlCentricity controlCentricity);
}

