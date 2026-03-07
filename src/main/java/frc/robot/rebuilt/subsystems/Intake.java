package frc.robot.rebuilt.subsystems;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.components.EasyMotor;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.subsystems.DashboardSubsystem;

public class Intake extends DashboardSubsystem {

    private SparkFlex intakeMotor;
    private SparkFlex deployMotor;
    private RelativeEncoder deployEncoder;
    private BooleanSupplier isStalling;

    public Intake(int intakeID, int deployID) {
        intakeMotor = EasyMotor.createEasySparkFlex(intakeID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kCoast);
        deployMotor = EasyMotor.createEasySparkFlex(deployID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
        deployEncoder = deployMotor.getEncoder();

        SparkFlexConfig deployConfig = new SparkFlexConfig();
        deployConfig.closedLoop.pid(0.1, 0, 0);
        deployConfig.closedLoop.outputRange(-1, 1);

        deployMotor.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        deployEncoder.setPosition(0);
    }

    public void intake() {
        intakeMotor.set(0.75);
    }

    public void outtake() {
        intakeMotor.set(-0.75);
    }

    public void stop() {
        intakeMotor.set(0.0);
    }

    public Command intakeCommand() {
        return Commands.runOnce(() -> intake());
    }

    public Command stopIntakeCommand() {
        return Commands.runOnce(() -> stop());
    }

    //TODO: change to actual encoder position
    public Command deploy() {
        return Commands.run(() -> RobotUtils.moveToPosition(deployMotor, 100)).until(isStalling);
    }


    public void stopDeploy() {
        deployMotor.set(0.0);
    }

    public Command manualDeploy() {
        return Commands.run(()->deployMotor.set(0.1));
    }

    public Command manualReverseDeploy() {
        return Commands.run(()->deployMotor.set(-0.1));
    }

    public Command reverseDeploy() {
        return Commands.run(() -> RobotUtils.moveToPosition(deployMotor, 0)).until(isStalling);
    }

    //TODO: replace with actual PDH channel and current
    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/Encoder/Position", deployEncoder.getPosition());
        double currentDraw = RobotUtils.getPDHCurrent(0);
        isStalling = RobotUtils.debounce(currentDraw, 35);
    }
}
