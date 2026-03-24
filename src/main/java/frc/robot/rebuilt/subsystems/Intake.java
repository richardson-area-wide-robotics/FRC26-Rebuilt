package frc.robot.rebuilt.subsystems;

import java.util.function.BooleanSupplier;

import frc.robot.common.annotations.NamedAuto;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.components.EasyMotor;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.subsystems.DashboardSubsystem;

public class Intake extends DashboardSubsystem {

    private SparkFlex intakeMotor1;
    private SparkFlex intakeMotor2;
    private SparkMax deployMotor1;
    //private SparkMax deployMotor2;
    private RelativeEncoder deployEncoder;
    private SparkMaxConfig deployLeaderConfig;
    //private SparkMaxConfig deployFollowerConfig;
    private BooleanSupplier isStalling;
    private boolean intakeRunning = false;

    public Intake(int intakeID1, int intakeID2, int deployID1) {
        intakeMotor1 = EasyMotor.createEasySparkFlex(intakeID1, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);
        intakeMotor2 = EasyMotor.createEasySparkFlex(intakeID2, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);
        deployMotor1 = EasyMotor.createEasySparkMax(deployID1, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
        //deployMotor2 = EasyMotor.createEasySparkMax(deployID2, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
        deployEncoder = deployMotor1.getEncoder();

        deployLeaderConfig = new SparkMaxConfig();
        deployLeaderConfig.closedLoop.pid(0.05, 0, 0);
        deployLeaderConfig.closedLoop.outputRange(-1, 1);
        deployMotor1.configure(deployLeaderConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        
        //deployFollowerConfig = new SparkMaxConfig();
        //deployFollowerConfig.closedLoop.pid(0.05, 0, 0);
        //deployFollowerConfig.closedLoop.outputRange(-1, 1);
        //deployFollowerConfig.follow(deployID1, true);
        //deployMotor2.configure(deployFollowerConfig,
        //    ResetMode.kResetSafeParameters,
        //    PersistMode.kPersistParameters);

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
        intakeMotor1.set(0.0);
        intakeMotor2.set(0.0);
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

    @NamedAuto(value = "Deploy Intake")
    public Command deploy() {
        return Commands.runOnce(() -> RobotUtils.moveToPosition(deployMotor1, 10));
    }


    public void stopDeploy() {
        deployMotor1.set(-0.05);
    }

    public void manualDeploy() {
        deployMotor1.set(0.2);
    }

    public void manualReverseDeploy() {
       deployMotor1.set(-0.25);
    }

    @NamedAuto(value = "Reverse Deploy Intake")
    public Command reverseDeploy() {
        return Commands.run(() -> RobotUtils.moveToPosition(deployMotor1, 0));
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
        Logger.recordOutput(getName() + "/Deploy/OutputCurrent", deployMotor1.getOutputCurrent());
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
