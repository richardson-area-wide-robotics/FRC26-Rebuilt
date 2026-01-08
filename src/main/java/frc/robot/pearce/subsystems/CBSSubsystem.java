package frc.robot.pearce.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.subsystems.DashboardSubsystem;


/**
 * A basic subsystem that only uses one motor
 * Provides methods for speed/stopping motors
 *
 * @author Hudson Strub
 * @since 2025
 */
public class CBSSubsystem extends DashboardSubsystem {
    public SparkFlex motor;
    private final RelativeEncoder encoder;

    public CBSSubsystem(int motorID) {
        SparkFlexConfig config = new SparkFlexConfig();

        config.closedLoop.pid(1, 0, 0);
        config.closedLoop.outputRange(-1, 1);

        motor = new SparkFlex(motorID, MotorType.kBrushless);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(100);
        motor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);        
        
        encoder = motor.getEncoder();
        encoder.setPosition(0); // Reset encoder position at startup
    }

    /**
     * Set the speed of the motor.
     * @param speed The speed to set (-1.0 to 1.0).
     */
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    /**
     * Stop the motor by setting its speed to 0.
     */
    public void stopMotor() {
        motor.set(0);
    }

    /**
     * Creates a command to set the shooter motor to a specific speed.
     * @param speed The speed to set (-1.0 to 1.0).
     * @return A command that sets the motor speed.
     */
    public Command setSpeedCommand(double speed) {
        return Commands.run(() -> setSpeed(speed), this);
    }

    /**
     * Creates a command to stop the shooter motor.
     * @return A command that stops the motor.
     */
    public Command stopMotorCommand() {
        return Commands.run(this::stopMotor, this);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public Command goToRest() {
        return Commands.run(() -> RobotUtils.moveToPosition(motor, 0.1));
    }

    public Command go45Degrees() {
        return Commands.run(() -> RobotUtils.moveToPosition(motor, 0.45));
    }
}