import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.common.components.hardware.swerve.MAXSwerveDrivetrain;
import frc.robot.common.components.hardware.swerve.MAXSwerveModule;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class MAXSwerveDrivetrainTest {

    private static MAXSwerveDrivetrain drivetrain;


    @BeforeAll
    static void setupOnce() {
        drivetrain = new MAXSwerveDrivetrain(
                1, 2, 0.0,
                3, 4, 0.0,
                5, 6, 0.0,
                7, 8, 0.0
        );
    }

    @BeforeEach
    void resetState() {
        drivetrain.resetEncoders();
    }

    @Test
    void testResetEncoders() {
        // set encoders to non-zero values
        drivetrain.getFrontLeft().getDrivingEncoder().setPosition(10.0);
        drivetrain.getFrontRight().getDrivingEncoder().setPosition(20.0);
        drivetrain.getRearLeft().getDrivingEncoder().setPosition(30.0);
        drivetrain.getRearRight().getDrivingEncoder().setPosition(40.0);

        drivetrain.resetEncoders();

        assertEquals(0.0, drivetrain.getFrontLeft().getDrivingEncoder().getPosition(), 1e-6);
        assertEquals(0.0, drivetrain.getFrontRight().getDrivingEncoder().getPosition(), 1e-6);
        assertEquals(0.0, drivetrain.getRearLeft().getDrivingEncoder().getPosition(), 1e-6);
        assertEquals(0.0, drivetrain.getRearRight().getDrivingEncoder().getPosition(), 1e-6);
    }

    @Test
    void testGetPositions() {
        SwerveModulePosition[] positions = drivetrain.getPositions();

        assertEquals(4, positions.length);
    }

    @Test
    void testSetXLock() {
        drivetrain.setXLock();

        SwerveModuleState[] states = drivetrain.getDesiredStates();

        for (int i = 0; i < states.length; i++) {
            assertEquals(0.0, states[i].speedMetersPerSecond, 1e-6,
                    "Module " + i + " should have zero speed");
        }

        // Angle pattern (X lock)
        assertEquals(Rotation2d.fromDegrees(45).getRadians(),
                states[0].angle.getRadians(), 1e-6);

        assertEquals(Rotation2d.fromDegrees(-45).getRadians(),
                states[1].angle.getRadians(), 1e-6);

        assertEquals(Rotation2d.fromDegrees(-45).getRadians(),
                states[2].angle.getRadians(), 1e-6);

        assertEquals(Rotation2d.fromDegrees(45).getRadians(),
                states[3].angle.getRadians(), 1e-6);
    }
}