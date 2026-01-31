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
    private final SparkClosedLoopController controller2;

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
        controller2 = motor2.getClosedLoopController();

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

    public void runShooterRPM(double rpm) {
        controller1.setSetpoint(-rpm, SparkBase.ControlType.kVelocity);
        //controller2.setSetpoint(rpm, SparkBase.ControlType.kVelocity);
    }

    public static int rpm = 4000;

    public void shoot(){
        runShooterRPM(-rpm);
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/Speed/DesiredRPM", rpm);
        Logger.recordOutput(getName() + "/Speed/RPM1", motor1.getEncoder().getVelocity());
        Logger.recordOutput(getName() + "/Speed/AMPS1", motor1.getOutputCurrent());
        Logger.recordOutput(getName() + "/Speed/RPM2", motor2.getEncoder().getVelocity());
    }

    public void stopShooter() {
        motor1.stopMotor();
        //motor2.stopMotor();
    }
}
