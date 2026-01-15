package frc.robot.pearce.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.EasyMotor;
import frc.robot.common.subsystems.DashboardSubsystem;

public class ProtoShooter extends DashboardSubsystem {

    private SparkFlex motor1;
    private SparkFlex motor2;


    public ProtoShooter(int id1, int id2){


         motor2 = EasyMotor.createEasySparkFlex(id1, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);
         motor1 = EasyMotor.createEasySparkFlex(id1, id2, false, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);

    }

    public void runShooter(){

        motor1.set(2);
    }
}
