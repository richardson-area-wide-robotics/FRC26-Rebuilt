package frc.robot.pearce.subsystems.MathSub;

import edu.wpi.first.units.Units;

public class MathShoot {

    private static final double G = Units.Gs.one().in(Units.MetersPerSecondPerSecond);
    private static final double hf = 75;  //change to meters
    private static final double hi = 29.67; //change to meters
    private static final double d = 4; //change to meters
    private static final double theta = 28;
    private static final double deltaH = hf-hi;


    public static double fetchLaunchSpeed(double distanceToTarget) {
        double num = (G * (d * d));
        double dom = (deltaH - d * Math.tan(theta) * 2 * (Math.cos(theta) * Math.cos(theta)));
        double Vo = Math.sqrt(num / dom);
        return Vo;

        //to do Check launch later


    }
}