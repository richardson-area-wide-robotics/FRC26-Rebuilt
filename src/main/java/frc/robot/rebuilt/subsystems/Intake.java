package frc.robot.rebuilt.subsystems;

import frc.robot.CommonConstants;
import frc.robot.common.annotations.NamedAuto;
import frc.robot.rebuilt.RebuiltConstants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
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
import frc.robot.common.components.EasyMotor;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.subsystems.DashboardSubsystem;

/**
 * Floor intake: two counter-rotating rollers on a motorised deploy arm.
 *
 * <p>Note the distinction between {@link #stopRollers()}, which genuinely stops, and
 * {@link #holdRollers()}, which applies a small retention bias. Both behaviours existed
 * before, but the biasing one was called {@code stop()}, which hid the fact that the rollers
 * were drawing current all match.
 */
public class Intake extends DashboardSubsystem {

    private final SparkFlex intakeMotor1;
    private final SparkFlex intakeMotor2;
    private final SparkMax deployMotor;
    private final RelativeEncoder deployEncoder;
    private boolean intakeRunning;

    public Intake(int intakeID1, int intakeID2, int deployID) {
        intakeMotor1 = new SparkFlex(intakeID1, SparkLowLevel.MotorType.kBrushless);
        intakeMotor2 = new SparkFlex(intakeID2, SparkLowLevel.MotorType.kBrushless);

        SparkFlexConfig intakeConfig1 = new SparkFlexConfig();
        intakeConfig1.idleMode(IdleMode.kCoast);
        intakeConfig1.smartCurrentLimit(CommonConstants.SUPERSTRUCTURE_CURRENT_LIMIT);
        intakeMotor1.configure(intakeConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig intakeConfig2 = new SparkFlexConfig();
        intakeConfig2.idleMode(IdleMode.kCoast);
        intakeConfig2.smartCurrentLimit(CommonConstants.SUPERSTRUCTURE_CURRENT_LIMIT);
        intakeMotor2.configure(intakeConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        deployMotor = EasyMotor.createEasySparkMax(deployID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
        deployEncoder = deployMotor.getEncoder();

        SparkMaxConfig deployConfig = new SparkMaxConfig();
        deployConfig.closedLoop.pid(IntakeConstants.DEPLOY_kP, 0, 0);
        deployConfig.closedLoop.outputRange(-1, 1);
        deployConfig.smartCurrentLimit(CommonConstants.SUPERSTRUCTURE_CURRENT_LIMIT);

        // Soft limits enforced by the controller itself, so they also bound open-loop
        // manual jogging. Previously these bounds existed only as commented-out checks in
        // periodic(), leaving nothing to stop the arm over-travelling into the frame.
        deployConfig.softLimit
            .reverseSoftLimit(IntakeConstants.DEPLOY_REVERSE_SOFT_LIMIT)
            .reverseSoftLimitEnabled(true)
            .forwardSoftLimit(IntakeConstants.DEPLOY_FORWARD_SOFT_LIMIT)
            .forwardSoftLimitEnabled(true);

        deployMotor.configure(deployConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        deployEncoder.setPosition(0);
    }

    public void intake() {
        intakeMotor1.set(-IntakeConstants.ROLLER_SPEED);
        intakeMotor2.set(IntakeConstants.ROLLER_SPEED);
        intakeRunning = true;
    }

    public void outtake() {
        intakeMotor1.set(IntakeConstants.ROLLER_SPEED);
        intakeMotor2.set(-IntakeConstants.ROLLER_SPEED);
        intakeRunning = true;
    }

    /**
     * Stops the rollers completely.
     *
     * <p>Use this whenever the intake should be genuinely idle — it draws no current.
     */
    public void stopRollers() {
        intakeMotor1.stopMotor();
        intakeMotor2.stopMotor();
        intakeRunning = false;
    }

    /**
     * Applies a light inward bias to keep a held game piece from falling out.
     *
     * <p>This is what the old {@code stop()} actually did. It is intentional, but it is not
     * a stop, and it costs current for as long as it is applied.
     */
    public void holdRollers() {
        intakeMotor1.set(-0.1);
        intakeMotor2.set(0.1);
        intakeRunning = false;
    }

    /** @return true while the rollers are commanded to intake or eject. */
    public boolean isRunning() {
        return intakeRunning;
    }

    /** @return deploy arm position in motor rotations. */
    public double getDeployPosition() {
        return deployEncoder.getPosition();
    }

    /** @return true when the deploy arm has reached the deployed position. */
    public boolean isDeployed() {
        return Math.abs(getDeployPosition() - IntakeConstants.DEPLOY_POSITION_ROTATIONS)
            <= IntakeConstants.DEPLOY_TOLERANCE_ROTATIONS;
    }

    /** @return true when the deploy arm has reached the stowed position. */
    public boolean isStowed() {
        return Math.abs(getDeployPosition() - IntakeConstants.STOW_POSITION_ROTATIONS)
            <= IntakeConstants.DEPLOY_TOLERANCE_ROTATIONS;
    }

    @NamedAuto(value = "Enable Intake")
    public Command intakeCommand() {
        return Commands.runOnce(this::intake, this);
    }

    @NamedAuto(value = "Disable Intake")
    public Command stopIntakeCommand() {
        return Commands.runOnce(this::holdRollers, this);
    }

    public Command deploy() {
        return Commands.runOnce(
            () -> RobotUtils.moveToPosition(deployMotor, IntakeConstants.DEPLOY_POSITION_ROTATIONS),
            this);
    }

    /**
     * Holds the deploy arm against its stop.
     *
     * <p>Not a true stop — see {@code IntakeConstants.DEPLOY_HOLD_SPEED}.
     */
    public void stopDeploy() {
        deployMotor.set(IntakeConstants.DEPLOY_HOLD_SPEED);
    }

    public void manualDeploy() {
        deployMotor.set(IntakeConstants.MANUAL_DEPLOY_SPEED);
    }

    public void manualReverseDeploy() {
        deployMotor.set(IntakeConstants.MANUAL_RETRACT_SPEED);
    }

    public Command reverseDeploy() {
        return Commands.run(
            () -> RobotUtils.moveToPosition(deployMotor, IntakeConstants.STOW_POSITION_ROTATIONS),
            this);
    }

    @NamedAuto(value = "Deploy Intake")
    public Command manualDeployCommand() {
        return Commands.runOnce(this::manualDeploy, this);
    }

    @NamedAuto(value = "Reverse Deploy Intake")
    public Command manualReverseDeployCommand() {
        return Commands.runOnce(this::manualReverseDeploy, this);
    }

    public Command jiggleItALittleCommand() {
        return Commands.repeatingSequence(
            RobotUtils.timedCommand(0.35, Commands.run(this::manualReverseDeploy), stopIntakeCommand()),
            RobotUtils.timedCommand(0.25, Commands.run(this::manualDeploy), stopIntakeCommand()));
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/Encoder/Position", getDeployPosition());
        Logger.recordOutput(getName() + "/Deploy/OutputCurrent", deployMotor.getOutputCurrent());
        Logger.recordOutput(getName() + "/Deploy/Deployed", isDeployed());
        Logger.recordOutput(getName() + "/Deploy/Stowed", isStowed());
        Logger.recordOutput(getName() + "/Activity/Intake", intakeRunning);
        Logger.recordOutput(getName() + "/Roller1/OutputCurrent", intakeMotor1.getOutputCurrent());
        Logger.recordOutput(getName() + "/Roller2/OutputCurrent", intakeMotor2.getOutputCurrent());
    }
}
