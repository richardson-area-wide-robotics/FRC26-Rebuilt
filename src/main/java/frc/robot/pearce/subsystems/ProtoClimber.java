package frc.robot.pearce.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.robot.common.components.EasyMotor;
import frc.robot.common.subsystems.DashboardSubsystem;
import org.littletonrobotics.junction.Logger;

//Will handel climbing
//Motor Count:
//Climber: 2
public class ProtoClimber extends DashboardSubsystem {
    private SparkFlex motor1;
    private SparkFlex motor2;
    private RelativeEncoder encoder;

    public ProtoClimber(int id1, int id2){
        motor1 = EasyMotor.createEasySparkFlex(id1, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
        motor2 = EasyMotor.createEasySparkFlex(id2, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
        encoder = motor1.getEncoder();
        }
    public void runClimber(){
        motor1.set(2);
        motor2.set(2);
    }

    public void stopClimber(){
        motor1.set(0);
        motor2.set(0);

    }
    @Override
    public void periodic(){
        Logger.recordOutput(getName()+"/Encoder/Position",encoder.getPosition());
        Logger.recordOutput(getName()+"/Encoder/Velocity",encoder.getVelocity());
    }
}
