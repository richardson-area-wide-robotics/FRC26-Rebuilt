package frc.robot.rebuilt.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.components.EasyMotor;
import frc.robot.common.subsystems.DashboardSubsystem;
import org.littletonrobotics.junction.Logger;

//Will handle getting balls from intake to shooter
//Motor Count:
// One (Tower)
public class Feeder extends DashboardSubsystem {

    private SparkMax feederMotor;
    private SparkMax spindexerMotor;

    public Feeder(int feederID, int spindexerID) {
        feederMotor = EasyMotor.createEasySparkMax(feederID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);
        spindexerMotor = EasyMotor.createEasySparkMax(spindexerID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);
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

        boolean spindexerRunning = spindexerMotor.get() > 0;
        boolean feederRunning = feederMotor.get() > 0;


        Logger.recordOutput(getName() + "/Activity/Spindexer", spindexerRunning);
        Logger.recordOutput(getName() + "/Activity/Feeder", feederRunning);

    }

    public Command loadAndCycleCommand() {
        return Commands.runOnce(() -> cycle()).alongWith(Commands.runOnce(() -> load()));
    }

    public Command stopLoadAndCycleCommand() {
        return Commands.runOnce(() -> stopCycle()).alongWith(Commands.runOnce(() -> stopLoad()));
    }
}
