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
import frc.robot.common.components.diagnostics.GamePieceCounter;
import frc.robot.common.components.diagnostics.MotorLoadMonitor;
import frc.robot.rebuilt.RebuiltConstants.FeederConstants;
import frc.robot.rebuilt.RebuiltConstants.LoadConstants;
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

    /**
     * Puts the feeder and spindexer in coast so they can be turned by hand, or back in brake.
     *
     * <p>For the hand-motion polarity check. Both are braked in normal operation, so both need this
     * before a hand can turn them far enough to read a direction.
     *
     * <p>No-persist, so a power cycle restores brake regardless of how the routine ended.
     *
     * <p><b>Entering coast also stops both motors.</b> Idle mode only governs a controller that is
     * applying nothing, so a running motor keeps running in coast — and a polarity reading taken from a
     * motor driving itself measures the motor, not the hand.
     *
     * @param coast True for coast, false to restore brake.
     */
    public void setCoastForHandCalibration(boolean coast) {
        if (coast) {
            feederMotor.stopMotor();
            spindexerMotor.stopMotor();
        }

        IdleMode mode = coast ? IdleMode.kCoast : IdleMode.kBrake;

        SparkFlexConfig feederConfig = new SparkFlexConfig();
        feederConfig.idleMode(mode);
        feederMotor.configure(feederConfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);

        SparkMaxConfig spindexerConfig = new SparkMaxConfig();
        spindexerConfig.idleMode(mode);
        spindexerMotor.configure(spindexerConfig, ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters);
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

    /** @return feeder current in amps, which is what a game piece passing through shows up in. */
    public double getFeederCurrent() {
        return feederMotor.getOutputCurrent();
    }

    /** @return feeder speed in motor RPM. */
    public double getFeederVelocity() {
        return feederMotor.getEncoder().getVelocity();
    }

    /** @return spindexer current in amps. */
    public double getSpindexerCurrent() {
        return spindexerMotor.getOutputCurrent();
    }

    /** @return spindexer speed in motor RPM. */
    public double getSpindexerVelocity() {
        return spindexerMotor.getEncoder().getVelocity();
    }

    /** Detects pieces and jams in the spindexer from its current and speed. */
    private final MotorLoadMonitor spindexerLoad = new MotorLoadMonitor(
        "Feeder/Spindexer",
        LoadConstants.SPINDEXER_WORK_EXCESS_AMPS,
        LoadConstants.SPINDEXER_EXPECTED_RPM,
        LoadConstants.JAM_SPEED_FRACTION,
        LoadConstants.JAM_CONFIRM_LOOPS);

    /**
     * Detects pieces and jams in the feeder.
     *
     * <p>The feeder is the vertical run up to the flywheel. <b>This is the motor the team calls the
     * tower motor</b> — confirmed, not inferred. There is no separate tower mechanism, and the name
     * kept here is {@code feeder}. It is a NEO Vortex on a SPARK Flex, 6784 RPM free.
     *
     * <p>Beware the collision: {@code ShooterPosition.TOWER} and {@code SectorType.TOWER} are a shot
     * preset and a field region respectively. Neither has anything to do with this motor.
     */
    private final MotorLoadMonitor feederLoad = new MotorLoadMonitor(
        "Feeder/Feeder",
        LoadConstants.FEEDER_WORK_EXCESS_AMPS,
        LoadConstants.FEEDER_EXPECTED_RPM,
        LoadConstants.JAM_SPEED_FRACTION,
        LoadConstants.JAM_CONFIRM_LOOPS);

    private final GamePieceCounter feederPieceCounter = new GamePieceCounter(
        "Feeder", feederLoad,
        LoadConstants.PIECE_SUSTAIN_LOOPS,
        LoadConstants.PIECE_REFRACTORY_LOOPS);

    /** @return true when the spindexer is loaded but not turning. */
    public boolean isSpindexerJammed() {
        return spindexerLoad.isJammed();
    }

    /** @return true when the feeder is loaded but not turning. */
    public boolean isFeederJammed() {
        return feederLoad.isJammed();
    }

    /** @return true when either mechanism is jammed. */
    public boolean isJammed() {
        return isSpindexerJammed() || isFeederJammed();
    }

    /** @return pieces detected past the feeder, which is the count that has reached the flywheel. */
    public int getPieceCount() {
        return feederPieceCounter.getCount();
    }

    /** Zeroes the piece count. */
    public void resetPieceCount() {
        feederPieceCounter.reset();
    }

    /** @return the spindexer load monitor, for jam clearing and diagnostics. */
    public MotorLoadMonitor getSpindexerLoad() {
        return spindexerLoad;
    }

    /** @return the feeder load monitor, for jam clearing and diagnostics. */
    public MotorLoadMonitor getFeederLoad() {
        return feederLoad;
    }

    @Override
    public void periodic() {
        // The spindexer holds at a crawl rather than stopping, so "commanded" means indexing at
        // speed — a crawl is not evidence of anything either way.
        spindexerLoad.update(getSpindexerCurrent(), getSpindexerVelocity(), isCycling());
        feederLoad.update(getFeederCurrent(), getFeederVelocity(), isLoading());
        feederPieceCounter.update();

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
