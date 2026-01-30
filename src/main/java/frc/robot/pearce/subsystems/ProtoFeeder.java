package frc.robot.pearce.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;

import frc.robot.common.components.EasyMotor;

//Will handle getting balls from intake to shooter
//Motor Count:
// One (Tower)
public class ProtoFeeder {

    private SparkFlex feederMotor;
    private SparkFlex spindexerMotor;

    public ProtoFeeder(int feederID, int spindexerID) {
        feederMotor = EasyMotor.createEasySparkFlex(feederID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);
        spindexerMotor = EasyMotor.createEasySparkFlex(spindexerID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);
    }

    public void cycle() {
        spindexerMotor.set(1.0);
    }

    public void load() {
        feederMotor.set(1.0);
    }
}
