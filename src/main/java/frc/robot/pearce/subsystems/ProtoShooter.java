package frc.robot.pearce.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkFlexConfig;
import frc.robot.common.components.EasyMotor;
import frc.robot.common.subsystems.DashboardSubsystem;
import org.littletonrobotics.junction.Logger;

public class ProtoShooter extends DashboardSubsystem {

    private final SparkFlex motor1;
    private final SparkFlex motor2;
    private final SparkClosedLoopController controller1;

    public ShooterPosition currentShooterPosition = ShooterPosition.AGAINST_HUB;

    public enum ShooterPosition {

        AGAINST_HUB(4000);

        public final float rpm;

        ShooterPosition(float rpm) {
            this.rpm = rpm;
        }
    }


    public ProtoShooter(int id1, int id2) {

        motor1 = EasyMotor.createEasySparkFlex(
                id1,
                SparkLowLevel.MotorType.kBrushless,
                SparkFlexConfig.IdleMode.kCoast
        );

        motor2 = EasyMotor.createEasySparkFlex(
                id2,
                SparkLowLevel.MotorType.kBrushless,
                SparkFlexConfig.IdleMode.kCoast
        );

        controller1 = motor1.getClosedLoopController();

        SparkFlexConfig config = new SparkFlexConfig();
        SparkFlexConfig config2 = new SparkFlexConfig();


        config.closedLoop
                .p(0.00035)
                .i(0.000001)
                .d(0.0065);

        config2.closedLoop
                .p(0.00035)
                .i(0.000001)
                .d(0.0065);

//        config.closedLoop.feedForward
//                .kS(5.565) //565000
//                .kV(0.001);
//                //.kA(1.5)
//                //.kG(0);


        config.smartCurrentLimit(65);
        config2.follow(motor1, true);

        motor1.configure(
                config,
                com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters
        );

        motor2.configure(
                config2,
                com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters
        );
    }

    public void runShooter(){
        controller1.setSetpoint(currentShooterPosition.rpm, SparkBase.ControlType.kVelocity);
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/RMP/DesiredRPM", currentShooterPosition.rpm);
        Logger.recordOutput(getName() + "/RMP/CurrentRPM", motor1.getEncoder().getVelocity());

        Logger.recordOutput(getName() + "/Motor1/CurrentAMPS", motor1.getOutputCurrent());
        Logger.recordOutput(getName() + "/Motor2/CurrentAMPS", motor2.getOutputCurrent());
    }

    public void stopShooter() {
        motor1.stopMotor();
    }
}
