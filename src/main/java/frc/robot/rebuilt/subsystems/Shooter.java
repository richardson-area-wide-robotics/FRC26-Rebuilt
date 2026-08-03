package frc.robot.rebuilt.subsystems;

import java.util.function.BooleanSupplier;

import frc.robot.CommonConstants;
import frc.robot.common.annotations.NamedAuto;
import frc.robot.common.subsystems.DashboardSubsystem;
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
    private final Spark motor2;

    /** True while this alliance's hub is scoring. Injected so it works in every mode. */
    private final BooleanSupplier hubActive;

    @Setter
    private ShooterPosition currentShooterPosition = ShooterPosition.HUB;

    /** Operator trim, in RPM, clamped to +/- {@link ShooterConstants#MAX_OPERATOR_TRIM_RPM}. */
    private double operatorRPMModifer;

    private boolean shooterRunning;

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

    /** @return the RPM the flywheel is currently being asked to hold. */
    public double getTargetRPM() {
        return currentShooterPosition.rpm + operatorRPMModifer;
    }

    /** @return the flywheel's measured RPM. */
    public double getMeasuredRPM() {
        return motor1.getInputs().analogVelocity;
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

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/Activity/Shooter", shooterRunning);
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
