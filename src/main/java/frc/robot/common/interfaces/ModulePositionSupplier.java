package frc.robot.common.interfaces;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * Supplies the current position of each swerve module.
 *
 * <p>Lets pose estimation depend on module positions without depending on a particular
 * drivetrain implementation.
 *
 * <p>Previously nested inside {@code AssumedPoseSubsystem}, which meant that class had to be
 * kept alive purely for this interface even after its own pose estimator was superseded.
 */
@FunctionalInterface
public interface ModulePositionSupplier {

    /** @return module positions, conventionally in FL, FR, RL, RR order. */
    SwerveModulePosition[] get();
}
