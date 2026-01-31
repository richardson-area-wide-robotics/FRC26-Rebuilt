package frc.robot.pearce.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.config.SparkBaseConfig;

import frc.robot.common.components.EasyMotor;
import frc.robot.common.subsystems.DashboardSubsystem;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class ProtoClimber extends DashboardSubsystem {

    private final SparkFlex motor1;
    //private final SparkFlex motor2;
    private final RelativeEncoder encoder;

    // AdvantageKit visualization
    private final LoggedMechanism2d climberMech;
    private final LoggedMechanismLigament2d climberLigament;

    private static final double METERS_PER_ROTATION = 0.3; // TODO we will need to adjust this when we have the phy climber
    private static final double MIN_LENGTH_METERS = 0.05;

    public ProtoClimber(int id1, int id2) {

        motor1 = EasyMotor.createEasySparkFlex(
                id1,
                SparkLowLevel.MotorType.kBrushless,
                SparkBaseConfig.IdleMode.kBrake
        );

//        motor2 = EasyMotor.createEasySparkFlex(
//                id2,
//                SparkLowLevel.MotorType.kBrushless,
//                SparkBaseConfig.IdleMode.kBrake
//        );

        encoder = motor1.getEncoder();

        climberMech = new LoggedMechanism2d(2.0, 2.0);

        LoggedMechanismRoot2d root =
                climberMech.getRoot("ClimberRoot", 1.0, 0.0);

        climberLigament = root.append(
                new LoggedMechanismLigament2d(
                        "Climber",
                        MIN_LENGTH_METERS,
                        90 // vertical
                )
        );

        Logger.recordOutput(getName()+"/Climber", climberMech);
    }

    public void runClimber() {
        motor1.set(0.3);
        //motor2.set(2);
    }
    public void unRunClimber() {
        motor1.set(-0.3);
        //motor2.set(2);
    }

    public void stopClimber() {
        motor1.set(0);
        //motor2.set(0);
    }

    @Override
    public void periodic() {

        double extensionMeters =
                Math.max(
                        MIN_LENGTH_METERS,
                        encoder.getPosition() * METERS_PER_ROTATION
                );

        // Update mechanism visualization
        climberLigament.setLength(extensionMeters);
        Logger.recordOutput(getName()+"/Climber", climberMech);


        // Log outputs
        Logger.recordOutput(
                getName() + "/ExtensionMeters",
                extensionMeters
        );
        Logger.recordOutput(
                getName() + "/Encoder/Position",
                encoder.getPosition()
        );
        Logger.recordOutput(
                getName() + "/Encoder/Velocity",
                encoder.getVelocity()
        );
    }
}