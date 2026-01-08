package frc.robot.common.subsystems.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.components.hardware.TankHardware;
import lombok.Getter;

import java.util.function.DoubleSupplier;

/**
 * Drive Subsystem for Tank Drive bots with 2 motors for each tread
 * This is unfinished
 *
 * @author Hudson Strub
 * @since 2025
 */
@Getter
public class TankDriveSubsystem extends SubsystemBase implements AutoCloseable {
    private final TankHardware TANK_HARDWARE;

    public TankDriveSubsystem(TankHardware hardware) {
        this.TANK_HARDWARE = hardware;
    }

    /**
     * Like that one Wii game
     *
     * @param forward  The speed to go forward
     * @param rotation The speed to rotate
     */
    public void arcadeDrive(double forward, double rotation) {
        double leftSpeed = forward + rotation;
        double rightSpeed = forward - rotation;
        TANK_HARDWARE.set(leftSpeed, rightSpeed);
    }

    public Command arcadeDriveCommand(DoubleSupplier forward, DoubleSupplier rotation) {
        return Commands.run(
                () -> arcadeDrive(-forward.getAsDouble(), rotation.getAsDouble()), // negative if forward stick is inverted
                this
        );
    }

    @Override
    public void close() {
        TANK_HARDWARE.close();
    }
}