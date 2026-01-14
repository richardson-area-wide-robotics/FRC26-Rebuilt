package frc.robot.kadiri.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.EasyMotor;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.kadiri.commands.RunIntake;


public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex intakeMotor;

    public IntakeSubsystem(int motorId) {
        intakeMotor = EasyMotor.createEasySparkFlex(
                motorId,
                SparkLowLevel.MotorType.kBrushless,
                SparkBaseConfig.IdleMode.kBrake
        );
    }

    public void intakeIn() {
        intakeMotor.set(1);
    }

    public void intakeOut() {

        intakeMotor.set(-1);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

}
