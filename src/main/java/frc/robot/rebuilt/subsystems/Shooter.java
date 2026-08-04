package frc.robot.rebuilt.subsystems;

import java.util.function.BooleanSupplier;

import frc.robot.CommonConstants;
import frc.robot.common.annotations.NamedAuto;
import frc.robot.common.components.diagnostics.GamePieceCounter;
import frc.robot.common.components.diagnostics.MotorLoadMonitor;
import frc.robot.common.components.diagnostics.TunableNumber;
import frc.robot.common.subsystems.DashboardSubsystem;
import frc.robot.rebuilt.RebuiltConstants.LoadConstants;
import frc.robot.rebuilt.RebuiltConstants.ShooterConstants;
import lombok.Setter;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.ID;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

/**
 * Two-motor flywheel shooter running closed-loop velocity, with a field-state interlock.
 *
 * <p>The shooter refuses to spin up while its alliance's HUB is inactive, so game pieces
 * aren't fired into a closed goal. That field state is supplied at construction rather than
 * read out of the container as a mutable static, which keeps the subsystem testable and —
 * more importantly — means the interlock is live in autonomous as well as teleop.
 */
public class Shooter extends DashboardSubsystem {

    private final Spark motor1;

    /**
     * The follower.
     *
     * <p>Deliberately retained but never read after construction: it mirrors the leader in
     * hardware, so nothing commands it directly. Keeping the reference means the object is
     * not garbage collected and stays available for diagnostics.
     */
    private final Spark motor2;

    /** True while this alliance's hub is scoring. Injected so it works in every mode. */
    private final BooleanSupplier hubActive;

    @Setter
    private ShooterPosition currentShooterPosition = ShooterPosition.HUB;

    /** Operator trim, in RPM, clamped to +/- {@link ShooterConstants#MAX_OPERATOR_TRIM_RPM}. */
    private double operatorRPMModifer;

    private boolean shooterRunning;

    /**
     * Live-tunable closed-loop gains.
     *
     * <p>Inert unless {@code TunableNumber.TUNING_ENABLED} is true, in which case these
     * appear on the dashboard and the SPARK is reconfigured only when a value actually
     * changes — reconfiguring every loop would flood the CAN bus.
     *
     * <p>The flywheel is the best candidate for live tuning on this robot: spin-up time and
     * recovery between shots dominate cycle time, and the effect of a gain change is visible
     * within a second on {@code Shooter/Activity/RPMError}.
     */
    private final TunableNumber tunableP = new TunableNumber("Shooter/kP", ShooterConstants.kP);
    private final TunableNumber tunableI = new TunableNumber("Shooter/kI", ShooterConstants.kI);
    private final TunableNumber tunableD = new TunableNumber("Shooter/kD", ShooterConstants.kD);

    public enum ShooterPosition {
        IDLE(1700),
        HUB(2100),
        TRENCH(3250),
        TOWER(2900),
        CORNER(4500);

        public final double rpm;

        ShooterPosition(double rpm) {
            this.rpm = rpm;
        }
    }

    /**
     * @param id1       CAN ID of the leader motor.
     * @param id2       CAN ID of the follower motor.
     * @param hubActive Supplies whether this alliance's hub is currently scoring.
     */
    public Shooter(int id1, int id2, BooleanSupplier hubActive) {
        this.hubActive = hubActive;

        motor1 = new Spark(
                new ID("ShooterHardware/ShooterLeader", id1),
                MotorKind.NEO_VORTEX,
                Units.Hertz.of(50)
        );

        motor2 = new Spark(
                new ID("ShooterHardware/ShooterFollower", id2),
                MotorKind.NEO_VORTEX,
                Units.Hertz.of(50)
        );

        // PID + current limit config
        SparkFlexConfig leaderConfig = new SparkFlexConfig();
        leaderConfig.idleMode(IdleMode.kCoast);
        leaderConfig.smartCurrentLimit(CommonConstants.SUPERSTRUCTURE_CURRENT_LIMIT);
        leaderConfig.closedLoop
                .p(ShooterConstants.kP)
                .i(ShooterConstants.kI)
                .d(ShooterConstants.kD);
        leaderConfig.inverted(false);

        motor1.configure(
                leaderConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );

        // Follower. It mirrors the leader's output directly, so it needs no closed-loop
        // gains of its own — configuring them here previously implied, misleadingly, that
        // the follower ran its own velocity loop.
        SparkFlexConfig followerConfig = new SparkFlexConfig();
        followerConfig.idleMode(IdleMode.kCoast);
        followerConfig.smartCurrentLimit(CommonConstants.SUPERSTRUCTURE_CURRENT_LIMIT);
        followerConfig.follow(id1, true);

        motor2.configure(
                followerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );
    }

    /**
     * Raises the operator trim, saturating at the configured limit.
     *
     * @param value RPM to add.
     */
    public void raiseOperatorModifer(double value) {
        setOperatorModifier(operatorRPMModifer + value);
    }

    /**
     * Lowers the operator trim, saturating at the configured limit.
     *
     * @param value RPM to subtract.
     */
    public void lowerOperatorModifer(double value) {
        setOperatorModifier(operatorRPMModifer - value);
    }

    /**
     * Sets the operator trim directly, clamped to the configured limit.
     *
     * <p>Previously this was unbounded, so repeated D-pad presses could drive the commanded
     * RPM arbitrarily high or negative.
     *
     * @param value Desired trim in RPM.
     */
    public void setOperatorModifier(double value) {
        operatorRPMModifer = MathUtil.clamp(
                value,
                -ShooterConstants.MAX_OPERATOR_TRIM_RPM,
                ShooterConstants.MAX_OPERATOR_TRIM_RPM);
    }

    /** @return the current operator trim in RPM. */
    public double getOperatorModifier() {
        return operatorRPMModifer;
    }

    /** Clears the operator trim back to zero. */
    public void resetOperatorModifier() {
        operatorRPMModifer = 0;
    }

    /** @return the position preset currently selected. */
    public ShooterPosition getCurrentShooterPosition() {
        return currentShooterPosition;
    }

    /**
     * Range-based target from the localisation state machine, when one is active.
     *
     * <p>{@code NaN} means no range target, in which case the selected preset applies. Kept
     * separate from the preset rather than overwriting it, so that when the assist disengages the
     * shooter returns to whatever the driver had chosen instead of being left on a stale number.
     */
    private double rangeTargetRpm = Double.NaN;

    /**
     * Sets a distance-derived target, overriding the preset while it is active.
     *
     * @param rpm Speed for the current range.
     */
    public void setRangeTargetRpm(double rpm) {
        rangeTargetRpm = rpm;
    }

    /** Clears the range target, handing the flywheel back to the selected preset. */
    public void clearRangeTarget() {
        rangeTargetRpm = Double.NaN;
    }

    /** @return true while a distance-derived target is overriding the preset. */
    public boolean hasRangeTarget() {
        return !Double.isNaN(rangeTargetRpm);
    }

    /**
     * @return the RPM the flywheel is currently being asked to hold — the range-based target
     *     when the aim assist is active, otherwise the selected preset. The operator trim
     *     applies either way, so a driver who knows the shot is running a little short can
     *     still bias it.
     */
    public double getTargetRPM() {
        double base = hasRangeTarget() ? rangeTargetRpm : currentShooterPosition.rpm;
        return base + operatorRPMModifer;
    }

    /**
     * @return the flywheel's measured RPM, from the motor's built-in encoder.
     *
     *     <p>This previously read {@code analogVelocity}. In PurpleLib that field is populated from
     *     {@code SparkBase.getAnalog().getVelocity()} — the <b>analog sensor on the SPARK's data
     *     port</b>, not the motor encoder. There is no analog sensor on either shooter motor, so it
     *     returned 0 on the real robot.
     *
     *     <p>The flywheel still span correctly, because the SPARK runs its velocity loop against its
     *     own primary encoder internally and never consults this value. What broke was everything
     *     built on top of it: {@link #isAtTargetRPM()} compared the target against 0 and so could
     *     never be true, and {@code Shooter/Activity/RPMError} logged the full setpoint as error for
     *     the whole match. Any "wait until up to speed" gate would have hung.
     *
     *     <p>{@code encoderVelocity} is the built-in encoder, which is what the closed loop already
     *     uses. Both fields are still logged so the shop session can confirm the analog channel is
     *     genuinely dead rather than merely assumed to be.
     */
    public double getMeasuredRPM() {
        return motor1.getInputs().encoderVelocity;
    }

    /** @return leader stator current in amps, which is what a game piece leaving shows up in. */
    public double getShooterCurrent() {
        return motor1.getInputs().statorCurrent.in(Units.Amps);
    }

    /** @return true while the flywheel is commanded to run. */
    public boolean isRunning() {
        return shooterRunning;
    }

    /** @return true when the measured RPM is within tolerance of the target. */
    public boolean isAtTargetRPM() {
        return shooterRunning
                && Math.abs(getTargetRPM() - getMeasuredRPM()) <= ShooterConstants.RPM_TOLERANCE;
    }

    /** @return true while the hub interlock permits shooting. */
    public boolean isHubActive() {
        return hubActive.getAsBoolean();
    }

    /**
     * Spins the flywheel to the current position's RPM, if the hub interlock allows it.
     *
     * <p>When the hub is inactive this is a no-op — the flywheel is left as it was rather
     * than being actively stopped, matching the previous behaviour.
     */
    public void runShooter() {
        if (!isHubActive()) {
            return;
        }
        shooterRunning = true;
        motor1.set(getTargetRPM(), ControlType.kVelocity);
    }

    public void stopShooter() {
        shooterRunning = false;
        motor1.stopMotor();
    }

    /**
     * Drops to an idle holding speed if the hub is live, otherwise stops entirely.
     *
     * <p>Keeping the flywheel spinning between shots costs current but removes spin-up time,
     * which matters when the hub is cycling.
     */
    public void idleOrStop() {
        if (ShooterConstants.IDLE_BETWEEN_SHOTS && isHubActive()) {
            setCurrentShooterPosition(ShooterPosition.IDLE);
            runShooter();
        } else {
            stopShooter();
        }
    }

    /**
     * Re-applies the closed-loop gains, used when a tunable value changes.
     *
     * @param p proportional gain.
     * @param i integral gain.
     * @param d derivative gain.
     */
    private void applyGains(double p, double i, double d) {
        SparkFlexConfig config = new SparkFlexConfig();
        config.closedLoop.p(p).i(i).d(d);
        // kNoResetSafeParameters and kNoPersistParameters: this is a live tuning tweak, not
        // a full reconfiguration, so leave everything else alone and do not burn flash.
        motor1.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        System.out.println("Shooter gains updated: p=" + p + " i=" + i + " d=" + d);
    }

    /**
     * Detects game pieces passing through the flywheel from current and RPM.
     *
     * <p>The flywheel is the one mechanism where the current signature is unambiguous. A piece
     * entering loads it hard and pulls the RPM down, then the closed loop drives current up to
     * recover. That excess-current-with-RPM-dip pattern is a shot, and nothing else on this
     * mechanism looks like it.
     *
     * <p>Expected speed is supplied live from {@link #getTargetRPM()} rather than fixed, because the
     * setpoint ranges from 1700 (IDLE) to 4500 (CORNER) RPM.
     */
    private final MotorLoadMonitor flywheelLoad = new MotorLoadMonitor(
            "Shooter/Flywheel",
            LoadConstants.SHOOTER_WORK_EXCESS_AMPS,
            this::getTargetRPM,
            LoadConstants.SHOOTER_JAM_SPEED_FRACTION,
            LoadConstants.JAM_CONFIRM_LOOPS);

    private final GamePieceCounter shotCounter = new GamePieceCounter(
            "Shooter", flywheelLoad,
            LoadConstants.PIECE_SUSTAIN_LOOPS,
            LoadConstants.SHOT_REFRACTORY_LOOPS);

    /** @return true while a game piece appears to be passing through the flywheel. */
    public boolean isShooting() {
        return flywheelLoad.isDoingWork();
    }

    /** @return true when the flywheel is loaded but not recovering speed — a wedged piece. */
    public boolean isJammed() {
        return flywheelLoad.isJammed();
    }

    /** @return shots detected since the last reset. A strong hint, not ground truth. */
    public int getShotCount() {
        return shotCounter.getCount();
    }

    /** Zeroes the shot count, e.g. at the start of a match. */
    public void resetShotCount() {
        shotCounter.reset();
    }

    /** @return the flywheel load monitor, for calibration and diagnostics. */
    public MotorLoadMonitor getFlywheelLoad() {
        return flywheelLoad;
    }

    @Override
    public void periodic() {
        // No-ops entirely when tuning is disabled.
        tunableP.ifChanged(p -> applyGains(p, tunableI.get(), tunableD.get()));
        tunableI.ifChanged(i -> applyGains(tunableP.get(), i, tunableD.get()));
        tunableD.ifChanged(d -> applyGains(tunableP.get(), tunableI.get(), d));

        flywheelLoad.update(getShooterCurrent(), getMeasuredRPM(), shooterRunning);
        shotCounter.update();

        Logger.recordOutput(getName() + "/Activity/Shooter", shooterRunning);
        Logger.recordOutput(getName() + "/Activity/ShotCount", getShotCount());

        // Both velocity fields, so the shop session can confirm the analog channel really is dead
        // rather than taking the claim in getMeasuredRPM()'s docs on trust.
        Logger.recordOutput(getName() + "/Sensors/EncoderRPM", motor1.getInputs().encoderVelocity);
        Logger.recordOutput(getName() + "/Sensors/AnalogRPM", motor1.getInputs().analogVelocity);
        Logger.recordOutput(getName() + "/Sensors/StatorAmps", getShooterCurrent());
        Logger.recordOutput(getName() + "/Activity/DesiredRPM", getTargetRPM());
        Logger.recordOutput(getName() + "/Activity/CurrentRPM", getMeasuredRPM());
        Logger.recordOutput(getName() + "/Activity/RPMError", getTargetRPM() - getMeasuredRPM());
        Logger.recordOutput(getName() + "/Activity/AtTarget", isAtTargetRPM());
        Logger.recordOutput(getName() + "/Activity/Position", currentShooterPosition.name());
        Logger.recordOutput(getName() + "/Activity/OperatorTrim", operatorRPMModifer);
        Logger.recordOutput(getName() + "/Interlock/HubActive", isHubActive());
    }

    @NamedAuto(value = "Enable Shooter")
    public Command runShooterCommand() {
        return Commands.runOnce(this::runShooter);
    }

    @NamedAuto(value = "Disable Shooter")
    public Command stopShooterCommand() {
        return Commands.runOnce(this::idleOrStop);
    }
}
