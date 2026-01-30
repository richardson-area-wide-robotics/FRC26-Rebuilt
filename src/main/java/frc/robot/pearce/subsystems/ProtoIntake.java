package frc.robot.pearce.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkHelpers.SparkModel;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.components.EasyMotor;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.subsystems.DashboardSubsystem;

//Will handle intaking
//Motor Count:
//Intake Spin: 1
//Intake Deploy: 1
public class ProtoIntake extends DashboardSubsystem {

    private SparkFlex intakeMotor;
    private SparkFlex deployMotor;
    private RelativeEncoder deployEncoder;

    public ProtoIntake(int intakeID, int deployID) {
        intakeMotor = EasyMotor.createEasySparkFlex(intakeID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
        deployMotor = EasyMotor.createEasySparkFlex(intakeID, SparkLowLevel.MotorType.kBrushless, SparkBaseConfig.IdleMode.kBrake);
        deployEncoder = deployMotor.getEncoder();

        deployEncoder.setPosition(0);

        SparkFlexConfig deployConfig = new SparkFlexConfig();
        deployConfig.closedLoop.pid(0.1, 0, 0);
        deployConfig.closedLoop.outputRange(-1, 1);

        deployMotor.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void intake() {
        intakeMotor.set(1.0);
    }

    public void outtake() {
        intakeMotor.set(-1.0);
    }

    //TODO: change to actual encoder position
    public Command deploy() {
        return Commands.run(() -> RobotUtils.moveToPosition(deployMotor, 100));
    }

    public Command reverseDeploy() {
        return Commands.run(() -> RobotUtils.moveToPosition(deployMotor, 0));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Deploy Motor Encoder Position", deployEncoder.getPosition());
    }
}
