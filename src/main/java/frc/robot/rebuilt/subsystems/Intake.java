package frc.robot.rebuilt.subsystems;

import frc.robot.common.annotations.NamedAuto;
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

public class Intake extends DashboardSubsystem {

    private SparkFlex intakeMotor1;
    private SparkFlex intakeMotor2;
    private SparkFlexConfig intakeConfig1;
    private SparkFlexConfig intakeConfig2;
    private SparkMax deployMotor;
    private RelativeEncoder deployEncoder;
    private SparkMaxConfig deployConfig;
    //private BooleanSupplier isStalling;
    private boolean intakeRunning = false;

    public Intake(int intakeID1, int intakeID2, int deployID) {
        intakeMotor1 = new SparkFlex(intakeID1, SparkLowLevel.MotorType.kBrushless);
        intakeMotor2 = new SparkFlex(intakeID2, SparkLowLevel.MotorType.kBrushless);

        intakeConfig1 = new SparkFlexConfig();
        intakeConfig1.idleMode(IdleMode.kCoast);
        intakeConfig1.smartCurrentLimit(60);
        intakeMotor1.configure(intakeConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeConfig2 = new SparkFlexConfig();
        intakeConfig2.idleMode(IdleMode.kCoast);
        intakeConfig2.smartCurrentLimit(60);
        intakeMotor2.configure(intakeConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        deployMotor = EasyMotor.createEasySparkMax(deployID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
        deployEncoder = deployMotor.getEncoder();

        deployConfig = new SparkMaxConfig();
        deployConfig.closedLoop.pid(0.05, 0, 0);
        deployConfig.closedLoop.outputRange(-1, 1);
        deployConfig.smartCurrentLimit(60);
        deployMotor.configure(deployConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        deployEncoder.setPosition(0);
    }

    public void intake() {
        intakeMotor1.set(-0.75);
        intakeMotor2.set(0.75);
        intakeRunning = true;
    }

    public void outtake() {
        intakeMotor1.set(0.75);
        intakeMotor2.set(-0.75);
        intakeRunning = true;
    }

    public void stop() {
        intakeMotor1.set(-0.1);
        intakeMotor2.set(0.1);
        intakeRunning = false;
    }

    @NamedAuto(value = "Enable Intake")
    public Command intakeCommand() {
        intakeRunning = true;
        return Commands.runOnce(() -> intake());
    }

    @NamedAuto(value = "Disable Intake")
    public Command stopIntakeCommand() {
        intakeRunning = false;
        return Commands.runOnce(() -> stop());
    }

    public Command deploy() {
        return Commands.runOnce(() -> RobotUtils.moveToPosition(deployMotor, 10));
    }

    public void stopDeploy() {
        deployMotor.set(-0.03);
    }

    public void manualDeploy() {
        deployMotor.set(0.2);
    }

    public void manualReverseDeploy() {
       deployMotor.set(-0.25);
    }

    public Command reverseDeploy() {
        return Commands.run(() -> RobotUtils.moveToPosition(deployMotor, 0));
    }

    @NamedAuto(value = "Deploy Intake")
    public Command manualDeployCommand() {
        return Commands.runOnce(() -> manualDeploy());
    }

    @NamedAuto(value = "Reverse Deploy Intake")
    public Command manualReverseDeployCommand() {
        return Commands.runOnce(() -> manualReverseDeploy());
    }

    public Command jiggleItALittleCommand() {
        return Commands.repeatingSequence(
            RobotUtils.timedCommand(0.35, Commands.run(this::manualReverseDeploy), stopIntakeCommand()),
            RobotUtils.timedCommand(0.25, Commands.run(this::manualDeploy), stopIntakeCommand()));
    }

    //TODO: replace with actual PDH channel and current
    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/Encoder/Position", deployEncoder.getPosition());
        Logger.recordOutput(getName() + "/Deploy/OutputCurrent", deployMotor.getOutputCurrent());
        Logger.recordOutput(getName() + "/Activity/Intake", intakeRunning);
        //if (deployEncoder.getPosition() < 0) {
        //    RobotUtils.moveToPosition(deployMotor, 0);
        //}
        //if (deployEncoder.getPosition() > 9) {
        //    RobotUtils.moveToPosition(deployMotor, 9);
        //}
    //    double currentDraw = RobotUtils.getPDHCurrent(0);
    //    isStalling = RobotUtils.debounce(currentDraw, 35);
    }
}
