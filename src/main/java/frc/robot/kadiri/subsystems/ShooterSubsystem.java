package frc.robot.kadiri.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.EasyMotor;




public class ShooterSubsystem extends SubsystemBase {

    private final SparkFlex shooterMotor;

    public ShooterSubsystem(int motorId) {
        shooterMotor = EasyMotor.createEasySparkFlex(
                2,
                SparkLowLevel.MotorType.kBrushless,
                SparkBaseConfig.IdleMode.kBrake
        );
    }

    public Command shooterIn() {

        shooterMotor.set(1);
        return null;
    }

    public Command shooterOut() {

        shooterMotor.set(-1);
        return null;
    }

    public Command stop() {
        shooterMotor.stopMotor();
        return null;
    }

}





