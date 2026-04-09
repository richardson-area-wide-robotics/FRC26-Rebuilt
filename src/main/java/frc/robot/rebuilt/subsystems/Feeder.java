package frc.robot.rebuilt.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.annotations.NamedAuto;
import frc.robot.common.components.EasyMotor;
import frc.robot.common.subsystems.DashboardSubsystem;
import org.littletonrobotics.junction.Logger;

public class Feeder extends DashboardSubsystem {

    private SparkFlex feederMotor;
    private SparkMax spindexerMotor;
    private SparkFlexConfig feederConfig;
    private SparkMaxConfig spindexerConfig;

    public Feeder(int feederID, int spindexerID) {

        feederConfig = new SparkFlexConfig();
        spindexerConfig = new SparkMaxConfig();

        feederMotor = new SparkFlex(feederID, SparkLowLevel.MotorType.kBrushless);
        feederConfig.idleMode(IdleMode.kBrake);
        feederConfig.smartCurrentLimit(60);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        spindexerMotor = new SparkMax(spindexerID, SparkLowLevel.MotorType.kBrushless);
        spindexerConfig.idleMode(IdleMode.kBrake);
        spindexerConfig.smartCurrentLimit(60);
        spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void cycle() {
        spindexerMotor.set(1.0);
    }

    public void load() {
        feederMotor.set(1.0);
    }

    public void stopCycle() {
        spindexerMotor.set(0.1);
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

    @NamedAuto(value = "Enable Load")
    public Command loadAndCycleCommand() {
        return Commands.runOnce(() -> cycle()).alongWith(Commands.runOnce(() -> load()));
    }
    @NamedAuto(value = "Disable Load")
    public Command stopLoadAndCycleCommand() {
        return Commands.runOnce(() -> stopCycle()).alongWith(Commands.runOnce(() -> stopLoad()));
    }
}
