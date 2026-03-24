package frc.robot.rebuilt.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import frc.robot.common.components.EasyMotor;
import frc.robot.common.subsystems.DashboardSubsystem;
import frc.robot.rebuilt.RebuiltContainer;
import lombok.Setter;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import org.littletonrobotics.junction.Logger;

public class Shooter extends DashboardSubsystem {

    private final SparkFlex motor1;
    private final SparkFlex motor2;

    @Setter
    private ShooterPosition currentShooterPosition = ShooterPosition.IDLE;
    private float operatorRPMModifer = 0;

    public void raiseOperatorModifer(float value){
        operatorRPMModifer += value;
    }
    public void lowerOperatorModifer(float value){
        operatorRPMModifer -= value;
    }

    public enum ShooterPosition {
        IDLE(1100),
        HUB(2100),
        TRENCH(3250),
        TOWER(2800),
        CORNER(4500);

        public final double rpm;

        ShooterPosition(double rpm) {
            this.rpm = rpm;
        }
    }

    public Shooter(int id1, int id2) {
        motor1 = new SparkFlex(id1, SparkLowLevel.MotorType.kBrushless);
        motor2 = new SparkFlex(id2, SparkLowLevel.MotorType.kBrushless);


        // PID + current limit config
        SparkFlexConfig leaderConfig = new SparkFlexConfig();
        leaderConfig.idleMode(IdleMode.kCoast);
        leaderConfig.smartCurrentLimit(65);
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

    boolean shooterRunning = false;


    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/Activity/Shooter", shooterRunning);
        Logger.recordOutput(getName() + "/Activity/DesiredRPM", currentShooterPosition.rpm + operatorRPMModifer);
        Logger.recordOutput(getName() + "/Activity/CurrentRPM",  motor1.getEncoder().getVelocity());

        shooterRunning = motor1.getEncoder().getVelocity() > 20;

        if (RebuiltContainer.hubOn) {
            motor1.set(
                    currentShooterPosition.rpm + operatorRPMModifer
            );
        }
        else{
            motor1.stopMotor();
        }

    }

//    @NamedAuto(value = "Enable Shooter")
//    public Command runShooterCommand() {
//        return Commands.runOnce(() -> runShooter());
//    }
//
//    @NamedAuto(value = "Disable Shooter")
//    public Command stopShooterCommand() {
//        return Commands.runOnce(() -> stopShooter());
//    }
}
