package frc.robot.rebuilt.subsystems;

import frc.robot.CommonConstants;
import frc.robot.common.subsystems.DashboardSubsystem;
import frc.robot.rebuilt.RebuiltConstants.ClimberConstants;

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

/**
 * Telescoping climber with an AdvantageKit mechanism visualisation.
 *
 * <p><strong>This subsystem is not currently on the robot.</strong> It is fully written but
 * never instantiated, and {@code RebuiltConstants.CanIds.CLIMBER_UNASSIGNED} has no real CAN
 * ID, so the robot has no climb capability. To enable it: assign a free CAN ID (9, 16 and 17
 * are unused), construct it in {@link frc.robot.rebuilt.RebuiltContainer}, and add driver
 * bindings for {@link #runClimber()} / {@link #unRunClimber()} / {@link #stopClimber()}.
 *
 * <p>{@code METERS_PER_ROTATION} is still a guess and must be measured on the real mechanism
 * before the visualisation or any position control means anything.
 */
public class Climber extends DashboardSubsystem {

    private final Spark motor1;

    // AdvantageKit visualization
    private final LoggedMechanism2d climberMech;
    private final LoggedMechanismLigament2d climberLigament;

    private static final double METERS_PER_ROTATION = ClimberConstants.METERS_PER_ROTATION;
    private static final double MIN_LENGTH_METERS = ClimberConstants.MIN_LENGTH_METERS;

    public Climber(int id1) {

        motor1 = new Spark(
                new ID("ClimberHardware/Climber", id1),
                MotorKind.NEO_VORTEX,
                Units.Hertz.of(50)
        );

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(CommonConstants.SUPERSTRUCTURE_CURRENT_LIMIT);

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

        Logger.recordOutput(getName() + "/Climber", climberMech);
    }

    public void runClimber() {
        motor1.set(ClimberConstants.CLIMB_SPEED);
    }

    public void unRunClimber() {
        motor1.set(-ClimberConstants.CLIMB_SPEED);
    }

    /** @return extension in metres, derived from the motor encoder. */
    public double getExtensionMeters() {
        return Math.max(MIN_LENGTH_METERS, motor1.getInputs().encoderPosition * METERS_PER_ROTATION);
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