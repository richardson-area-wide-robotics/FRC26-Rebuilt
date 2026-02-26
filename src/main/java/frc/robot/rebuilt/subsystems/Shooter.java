package frc.robot.rebuilt.subsystems;

import frc.robot.common.subsystems.DashboardSubsystem;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.ID;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.units.Units;

public class Shooter extends DashboardSubsystem {

    private final Spark motor1;
    private final Spark motor2;

    public ShooterPosition currentShooterPosition = ShooterPosition.AGAINST_HUB;

    public enum ShooterPosition {
        AGAINST_HUB(2437.5);

        public final double rpm;

        ShooterPosition(double rpm) {
            this.rpm = rpm;
        }
    }

    public Shooter(int id1, int id2) {
        motor1 = new Spark(
                new ID("ShooterHardware/ShooterLeader", id1),
                MotorKind.NEO_VORTEX,
                Units.Hertz.of(50)
        );

        motor2 = new Spark(
                new ID("ShooterHardware/ShooterFollower", id2),
                MotorKind.NEO_VORTEX,
                Units.Hertz.of(50)
        );

        // PID + current limit config
        SparkFlexConfig leaderConfig = new SparkFlexConfig();
        leaderConfig.idleMode(IdleMode.kCoast);
        leaderConfig.smartCurrentLimit(65);
        leaderConfig.closedLoop
                .p(0.00035)
                .i(0.000001)
                .d(0.0065);

        motor1.configure(
                leaderConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );

        // Follower
        SparkFlexConfig followerConfig = new SparkFlexConfig();
        followerConfig.idleMode(IdleMode.kCoast);
        followerConfig.smartCurrentLimit(65);
        followerConfig.closedLoop
                .p(0.00035)
                .i(0.000001)
                .d(0.0065);

                followerConfig.follow(id1, true);
        motor2.configure(
                followerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );
    }

    public void runShooter() {
        motor1.set(
                currentShooterPosition.rpm,
                ControlType.kVelocity
        );
    }

    public void stopShooter() {
        motor1.stopMotor();
    }

}
