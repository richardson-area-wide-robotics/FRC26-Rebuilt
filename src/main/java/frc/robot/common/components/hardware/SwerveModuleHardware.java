package frc.robot.common.components.hardware;

import org.lasarobotics.hardware.revrobotics.Spark;

/**
 * Swerve module hardware for a single REV swerve module
 *
 * @author Hudson Strub
 * @since 2025 Offseason
 */
public record SwerveModuleHardware(Spark driveMotor, Spark rotateMotor) {

}

