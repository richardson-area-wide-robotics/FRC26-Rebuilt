package frc.robot.pearce.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.components.EasyMotor;
import frc.robot.common.subsystems.DashboardSubsystem;

public class DeepClimbSubsystem extends DashboardSubsystem {

    public SparkFlex motor;
    public SparkFlex motor2;

    public DeepClimbSubsystem(int motorID, int motor2ID) {
        motor = EasyMotor.createEasySparkFlex(motorID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
        motor2 = EasyMotor.createEasySparkFlex(motor2ID, motorID, true, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);

    }

    public Command in() {
        return Commands.run(() -> motor.set(0.3), this);
    }
    public Command out() {
        return Commands.run(() -> motor.set(-0.3), this);
    }
    public Command stop(){
        return Commands.run(() -> motor.set(0), this);
    }
}
