package frc.robot.rebuilt.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CommonConstants;
import frc.robot.common.annotations.NamedAuto;
import frc.robot.common.subsystems.DashboardSubsystem;
import frc.robot.rebuilt.RebuiltConstants.FeederConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Indexing and feeding: a spindexer that settles game pieces, and a feeder that pushes them
 * into the flywheel.
 *
 * <p>{@link #holdCycle()} applies a deliberate slow crawl rather than stopping, which keeps
 * pieces from wedging. It was previously named {@code stopCycle()}, which obscured that.
 * {@link #stopCycle()} now genuinely stops.
 */
public class Feeder extends DashboardSubsystem {

    private final SparkFlex feederMotor;
    private final SparkMax spindexerMotor;

    private double feederDemand;
    private double spindexerDemand;

    public Feeder(int feederID, int spindexerID) {
        SparkFlexConfig feederConfig = new SparkFlexConfig();
        SparkMaxConfig spindexerConfig = new SparkMaxConfig();

        feederMotor = new SparkFlex(feederID, SparkLowLevel.MotorType.kBrushless);
        feederConfig.idleMode(IdleMode.kBrake);
        feederConfig.smartCurrentLimit(CommonConstants.SUPERSTRUCTURE_CURRENT_LIMIT);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        spindexerMotor = new SparkMax(spindexerID, SparkLowLevel.MotorType.kBrushless);
        spindexerConfig.idleMode(IdleMode.kBrake);
        spindexerConfig.smartCurrentLimit(CommonConstants.SUPERSTRUCTURE_CURRENT_LIMIT);
        spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setSpindexer(double demand) {
        spindexerDemand = demand;
        spindexerMotor.set(demand);
    }

    private void setFeeder(double demand) {
        feederDemand = demand;
        feederMotor.set(demand);
    }

    public void cycle() {
        setSpindexer(FeederConstants.SPINDEXER_SPEED);
    }

    public void load() {
        setFeeder(FeederConstants.FEEDER_SPEED);
    }

    /**
     * Slows the spindexer to its retention crawl.
     *
     * <p>This is what the old {@code stopCycle()} did. Kept because it stops pieces wedging,
     * but named honestly.
     */
    public void holdCycle() {
        setSpindexer(FeederConstants.SPINDEXER_HOLD_SPEED);
    }

    /** Stops the spindexer completely. */
    public void stopCycle() {
        spindexerDemand = 0;
        spindexerMotor.stopMotor();
    }

    /** Stops the feeder completely. */
    public void stopLoad() {
        setFeeder(0.0);
    }

    public void reverseCycle() {
        setSpindexer(-FeederConstants.SPINDEXER_SPEED);
    }

    public void reverseLoad() {
        setFeeder(-FeederConstants.FEEDER_SPEED);
    }

    /** @return the demand last sent to the feeder motor. */
    public double getFeederDemand() {
        return feederDemand;
    }

    /** @return the demand last sent to the spindexer motor. */
    public double getSpindexerDemand() {
        return spindexerDemand;
    }

    /** @return true while the feeder is driving pieces toward the flywheel. */
    public boolean isLoading() {
        return feederDemand > 0;
    }

    /** @return true while the spindexer is indexing at speed, rather than merely holding. */
    public boolean isCycling() {
        return spindexerDemand > FeederConstants.SPINDEXER_HOLD_SPEED;
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/Activity/Spindexer", isCycling());
        Logger.recordOutput(getName() + "/Activity/Feeder", isLoading());
        Logger.recordOutput(getName() + "/Demand/Spindexer", spindexerDemand);
        Logger.recordOutput(getName() + "/Demand/Feeder", feederDemand);
        Logger.recordOutput(getName() + "/Feeder/OutputCurrent", feederMotor.getOutputCurrent());
        Logger.recordOutput(getName() + "/Spindexer/OutputCurrent", spindexerMotor.getOutputCurrent());
    }

    @NamedAuto(value = "Enable Load")
    public Command loadAndCycleCommand() {
        return Commands.runOnce(() -> {
            cycle();
            load();
        }, this);
    }

    @NamedAuto(value = "Disable Load")
    public Command stopLoadAndCycleCommand() {
        return Commands.runOnce(() -> {
            holdCycle();
            stopLoad();
        }, this);
    }
}
