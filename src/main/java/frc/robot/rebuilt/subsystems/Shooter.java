package frc.robot.rebuilt.subsystems;

import frc.robot.common.components.RobotUtils;
import frc.robot.common.subsystems.DashboardSubsystem;
import lombok.Setter;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.ID;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.units.Units;
import org.littletonrobotics.junction.Logger;

public class Shooter extends DashboardSubsystem {

    private final Spark motor1;
    private final Spark motor2;

    @Setter
    private ShooterPosition currentShooterPosition = ShooterPosition.AGAINST_HUB;
    public float operatorRMPModifer = 0;

    public enum ShooterPosition {
        AGAINST_HUB(2438),
        CORNER(2438),
        CLIMBER(2438);

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
        leaderConfig.inverted(true);

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
        shooterRunning = true;
        motor1.set(
                currentShooterPosition.rpm + operatorRMPModifer,
                ControlType.kVelocity
        );
    }

    boolean shooterRunning = false;

    public void stopShooter() {
        shooterRunning = false;
        motor1.stopMotor();
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/Activity/Shooter", shooterRunning);
    }

}
