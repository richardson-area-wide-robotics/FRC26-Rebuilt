package frc.robot.common.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

@FunctionalInterface
public interface ModulePositionSupplier {
    SwerveModulePosition[] getSwerveModulePositions();
}
