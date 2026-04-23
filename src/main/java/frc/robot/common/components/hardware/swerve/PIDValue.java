package frc.robot.common.components.hardware.swerve;

import lombok.Getter;

public class PIDValue {

    @Getter private final double kP;
    @Getter private final double kI;
    @Getter private final double kD;

    public PIDValue(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
