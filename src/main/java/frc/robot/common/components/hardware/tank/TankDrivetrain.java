package frc.robot.common.components.hardware.tank;

import com.revrobotics.spark.SparkBase;
import frc.robot.common.components.hardware.gyro.IMU;

import java.util.List;

/**
 * Drivetrain for a robot with tank drive
 *
 * Supports multiple left and right motors
 *
 * @author Hudson Strub
 * @since 2025 Offseason
 */
public record TankDrivetrain(IMU gyro, List<SparkBase> lMotors, List<SparkBase> rMotors) implements AutoCloseable  {


    public void set(double leftSpeed, double rightSpeed) {
        lMotors.forEach(m -> m.set(leftSpeed));
        rMotors.forEach(m -> m.set(rightSpeed));
    }

    public void stop() {
        lMotors.forEach(m -> m.set(0.0));
        rMotors.forEach(m -> m.set(0.0));
    }

    @Override
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
