package frc.robot.rebuilt.subsystems;

import frc.robot.common.components.NamedAutoRegistry;
import frc.robot.common.subsystems.DashboardSubsystem;
import org.lasarobotics.hardware.revrobotics.Spark;
import org.lasarobotics.hardware.revrobotics.Spark.ID;
import org.lasarobotics.hardware.revrobotics.Spark.MotorKind;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.units.Units;

public class Climber extends DashboardSubsystem {

    private final Spark motor1;
    // private final Spark motor2;

    // AdvantageKit visualization
    private final LoggedMechanism2d climberMech;
    private final LoggedMechanismLigament2d climberLigament;

    // Physics Constants - TODO adjust these when physical hardware is available
    private static final double GEAR_RATIO = 12.0; // 12:1 reduction (Example)
    private static final double DRUM_DIAMETER_METERS = 0.025; // 1 inch drum (Example)
    private static final double DRUM_CIRCUMFERENCE = DRUM_DIAMETER_METERS * Math.PI;

    private static final double METERS_PER_ROTATION = DRUM_CIRCUMFERENCE / GEAR_RATIO;
    private static final double MIN_LENGTH_METERS = 0.05;

    public Climber(int id1) {

        motor1 = new Spark(
                new ID("ClimberHardware/Climber", id1),
                MotorKind.NEO_VORTEX,
                Units.Hertz.of(50)
        );

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);

        motor1.configure(
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
        );

        climberMech = new LoggedMechanism2d(2.0, 2.0);

        LoggedMechanismRoot2d root =
                climberMech.getRoot("ClimberRoot", 1.0, 0.0);

        climberLigament = root.append(
                new LoggedMechanismLigament2d(
                        "Climber",
                        MIN_LENGTH_METERS,
                        90
                )
        );

        // Register @NamedAuto commands manually to improve boot-up speed
        NamedAutoRegistry.register(this);

        Logger.recordOutput(getName() + "/Climber", climberMech);
    }

    public void runClimber() {
        motor1.set(0.3);
    }

    public void unRunClimber() {
        motor1.set(-0.3);
    }

    public void stopClimber() {
        motor1.stopMotor();
    }

    @Override
    public void periodic() {

        var inputs = motor1.getInputs();

        double extensionMeters =
                Math.max(
                        MIN_LENGTH_METERS,
                        inputs.encoderPosition * METERS_PER_ROTATION
                );

        // Update mechanism visualization
        climberLigament.setLength(extensionMeters);
        Logger.recordOutput(getName() + "/Climber", climberMech);

        // Log outputs
        Logger.recordOutput(getName() + "/ExtensionMeters", extensionMeters);
    }
}