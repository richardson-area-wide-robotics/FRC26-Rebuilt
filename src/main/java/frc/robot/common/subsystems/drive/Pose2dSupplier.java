package frc.robot.common.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;

@FunctionalInterface
public interface Pose2dSupplier {
    Pose2d getPose();
}
