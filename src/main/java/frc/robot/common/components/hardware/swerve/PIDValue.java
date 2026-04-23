package frc.robot.common.components.hardware.swerve;

import lombok.Getter;

public class PIDValue {

    @Getter private double kP;
    @Getter private double kI;
    @Getter private double kD;

    public PIDValue(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
