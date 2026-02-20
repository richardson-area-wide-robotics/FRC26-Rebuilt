package frc.robot.pearce.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import frc.robot.common.components.EasyMotor;

//Will handle getting balls from intake to shooter
//Motor Count:
// One (Tower)
public class Feeder {

    private SparkMax feederMotor;
    private SparkFlex spindexerMotor;

    public Feeder(int feederID, int spindexerID) {
        feederMotor = EasyMotor.createEasySparkMax(feederID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);
        spindexerMotor = EasyMotor.createEasySparkFlex(spindexerID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);
    }

    public void cycle() {
        spindexerMotor.set(1.0);
    }

    public void load() {
        feederMotor.set(1.0);
    }

    public void stopCycle() {
        spindexerMotor.set(0.0);
    }

    public void stopLoad() {
        feederMotor.set(0.0);
    }

    public void reverseCycle() {
        spindexerMotor.set(-1.0);
    }

    public void reverseLoad() {
        feederMotor.set(-1.0);
    }
}
