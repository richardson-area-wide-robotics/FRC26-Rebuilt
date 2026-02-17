package frc.robot.pearce.subsystems.MathSub;

import edu.wpi.first.units.Units;

public class MathShoot {

    private static final double G =
            Units.Gs.one().in(Units.MetersPerSecondPerSecond);

    // Heights (meters)
    private static final double hf = 0.75;    // target height
    private static final double hi = 0.2967;  // shooter height

    // Launch angle (radian)
    private static final double theta = Math.toRadians(28);

    private static final double deltaH = hf - hi;

    public static double fetchLaunchSpeed(double distanceToTarget) {

        double numerator = G * (distanceToTarget * distanceToTarget);

        double denominator =
                2 * Math.pow(Math.cos(theta), 2) *
                        (distanceToTarget * Math.tan(theta) - deltaH);

        return Math.sqrt(numerator / denominator);
    }
}