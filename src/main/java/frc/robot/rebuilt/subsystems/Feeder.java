package frc.robot.rebuilt.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import frc.robot.common.components.EasyMotor;
import frc.robot.common.subsystems.DashboardSubsystem;
import org.littletonrobotics.junction.Logger;

//Will handle getting balls from intake to shooter
//Motor Count:
// One (Tower)
public class Feeder extends DashboardSubsystem {

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


    @Override
    public void periodic() {

        boolean spindexterRunning = spindexerMotor.get() > 0;
        boolean feederRunning = feederMotor.get() > 0;


        Logger.recordOutput(getName() + "/Activity/Spindexter", spindexterRunning);
        Logger.recordOutput(getName() + "/Activity/Feeder", feederRunning);

    }

}
