package frc.robot.common.components.hardware;

import com.revrobotics.spark.SparkBase;
import frc.robot.common.interfaces.IMU;

import java.util.List;

/**
 * Drive hardware for a robot with tank drive
 *
 * Supports multiple left and right motors
 *
 * @author Hudson Strub
 * @since 2025 Offseason
 */
public record TankHardware(IMU gyro, List<SparkBase> lMotors, List<SparkBase> rMotors) {


    public void set(double leftSpeed, double rightSpeed) {
        lMotors.forEach(m -> m.set(leftSpeed));
        rMotors.forEach(m -> m.set(rightSpeed));
    }

    public void stop() {
        lMotors.forEach(m -> m.set(0.0));
        rMotors.forEach(m -> m.set(0.0));
    }

    public void close() {
        try {
            gyro.close();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        lMotors.forEach(SparkBase::close);
        rMotors.forEach(SparkBase::close);
    }
}
