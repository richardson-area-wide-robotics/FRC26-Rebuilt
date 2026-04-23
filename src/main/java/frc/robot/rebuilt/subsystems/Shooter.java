package frc.robot.rebuilt.subsystems;

import frc.robot.common.annotations.NamedAuto;
import frc.robot.common.subsystems.DashboardSubsystem;
import frc.robot.rebuilt.RebuiltContainer;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

public class Shooter extends DashboardSubsystem {

    private final Spark motor1;
    private final Spark motor2;

    @Setter
    private ShooterPosition currentShooterPosition = ShooterPosition.HUB;
    private float operatorRPMModifer = 0;

    public void raiseOperatorModifer(float value){
        operatorRPMModifer += value;
    }
    public void lowerOperatorModifer(float value){
        operatorRPMModifer -= value;
    }

    public enum ShooterPosition {
        IDLE(1700),
        HUB(2100),
        TRENCH(3250),
        TOWER(2900),
        CORNER(4500);

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
        leaderConfig.smartCurrentLimit(60);
        leaderConfig.closedLoop
                .p(0.00035)
                .i(0.000001)
                .d(0.0065);
        leaderConfig.inverted(false);

        motor1.configure(
                leaderConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );

        // Follower
        SparkFlexConfig followerConfig = new SparkFlexConfig();
        followerConfig.idleMode(IdleMode.kCoast);
        followerConfig.smartCurrentLimit(60);
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
        if (RebuiltContainer.hubOn) {
            shooterRunning = true;
            motor1.set(
                currentShooterPosition.rpm + operatorRPMModifer,
                ControlType.kVelocity
            );
        }
    }

    boolean shooterRunning = false;

    public void stopShooter() {
        shooterRunning = false;
        motor1.stopMotor();
    }

    private final boolean gonnaUseIdle = true;

    public void idleOrStop(){
        if(gonnaUseIdle && (RebuiltContainer.hubOn || RebuiltContainer.hubBlinking)){
            setCurrentShooterPosition(ShooterPosition.IDLE);
            runShooter();
        }
        else{
            stopShooter();
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/Activity/Shooter", shooterRunning);
        Logger.recordOutput(getName() + "/Activity/DesiredRPM", currentShooterPosition.rpm + operatorRPMModifer);
        Logger.recordOutput(getName() + "/Activity/CurrentRPM",  motor1.getInputs().analogVelocity);
    }

    @NamedAuto(value = "Enable Shooter")
    public Command runShooterCommand() {
        return Commands.runOnce(() -> runShooter());
    }

    @NamedAuto(value = "Disable Shooter")
    public Command stopShooterCommand() {
        return Commands.runOnce(() -> idleOrStop());
    }
}
