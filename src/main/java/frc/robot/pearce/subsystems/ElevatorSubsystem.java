package frc.robot.pearce.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import  com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.annotations.NamedAuto;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.subsystems.DashboardSubsystem;

public class ElevatorSubsystem extends DashboardSubsystem {

    private final SparkFlex motor;
    private final RelativeEncoder encoder;

    private static final double BOTTOM_POSITION = 0.2;
    private static final double L1_POSITION =  16.0;
    private static final double L2_POSITION = 25.3;
    private static final double L3_POSITION = 40.5;
    private static final double INTAKE_POSITION = 13.6;

    public ElevatorSubsystem(int motorID) {
       SparkFlexConfig config = new SparkFlexConfig();

       config.closedLoop.p(0.1).i(0).d(0).outputRange(-1, 1);

       motor = new SparkFlex(motorID, MotorType.kBrushless);
       config.idleMode(IdleMode.kBrake);
       motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        
       encoder = motor.getEncoder();
       encoder.setPosition(0); // Reset encoder position at startup
    }

    public Command up() {
        return Commands.run(() -> motor.set(0.4), this);
    }

    public Command down() {
        return Commands.run(() -> motor.set(-0.4), this);
    }

    public Command stop() {
        return Commands.run(() -> motor.set(0.03), this);
    }

    public Command goToBottom() {
        return Commands.run(() -> RobotUtils.moveToPosition(motor, BOTTOM_POSITION));
    }

    public Command goLevelOne() {
        return Commands.run(() -> RobotUtils.moveToPosition(motor, L1_POSITION));
    }

    public Command goLevelTwo() {
        return Commands.run(() -> RobotUtils.moveToPosition(motor, L2_POSITION));
    }

    public Command goLevelThree() {
        return Commands.run(() -> RobotUtils.moveToPosition(motor, L3_POSITION));
    }

    public Command goToIntake() {
        return Commands.run(() -> RobotUtils.moveToPosition(motor, INTAKE_POSITION));
    }

    @NamedAuto("Elevator Bottom")
    public Command autoGoToBottom() {
        return Commands.runOnce(() -> RobotUtils.moveToPosition(motor, BOTTOM_POSITION));
    }

    @NamedAuto("Elevator L1")
    public Command autoGoLevelOne() {
        return Commands.runOnce(() -> RobotUtils.moveToPosition(motor, L1_POSITION));
    }

    @NamedAuto("Elevator L2")
    public Command autoGoLevelTwo() {
        return Commands.runOnce(() -> RobotUtils.moveToPosition(motor, L2_POSITION));
    }

    @NamedAuto("Elevator L3")
    public Command autoGoLevelThree() {
        return Commands.runOnce(() -> RobotUtils.moveToPosition(motor, L3_POSITION));
    }

    @NamedAuto("Elevator Intake")
    public Command autoGoToIntake() {
        return Commands.runOnce(() -> RobotUtils.moveToPosition(motor, INTAKE_POSITION));
    }

    public Command resetEncoder() {
        return Commands.runOnce(() -> encoder.setPosition(0));
    }
}
