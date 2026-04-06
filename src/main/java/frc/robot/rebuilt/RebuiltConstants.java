package frc.robot.rebuilt;


/**
 * The RebuiltConstants class provides a convenient place for teams to hold game-specific
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public class RebuiltConstants {

  public static class HardwareConstants {
    // Shooter
    public static final int SHOOTER_LEADER_ID = 10;
    public static final int SHOOTER_FOLLOWER_ID = 11;

    // Feeder
    public static final int FEEDER_MOTOR_ID = 18;
    public static final int SPINDEXER_MOTOR_ID = 14;

    // Intake
    public static final int INTAKE_MOTOR_1_ID = 13;
    public static final int INTAKE_MOTOR_2_ID = 15;
    public static final int INTAKE_DEPLOY_MOTOR_ID = 12;
  }

}
