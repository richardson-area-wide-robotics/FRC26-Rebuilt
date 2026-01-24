package frc.robot.pearce.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.common.components.EasyMotor;
import frc.robot.common.subsystems.DashboardSubsystem;

public class ProtoShooter extends DashboardSubsystem {

    private SparkFlex motor1;
    private SparkFlex motor2;


    public ProtoShooter(int id1, int id2){
         motor1 = EasyMotor.createEasySparkFlex(id1, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);
         motor2 = EasyMotor.createEasySparkFlex(id2, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);
    }

    public void runShooter(){
        motor1.set(-2);
        motor2.set(2);
    }

    public void stopShooter(){
        motor1.set(0);
        motor2.set(0);

    }
}
