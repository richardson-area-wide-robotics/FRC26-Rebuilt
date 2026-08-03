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
public final class RebuiltConstants {

  private RebuiltConstants() {
  }

  /** CAN IDs for the 2026 superstructure. Swerve IDs live in {@code CommonConstants}. */
  public static final class CanIds {
    public static final int SHOOTER_LEADER = 10;
    public static final int SHOOTER_FOLLOWER = 11;
    public static final int INTAKE_DEPLOY = 12;
    public static final int INTAKE_ROLLER_1 = 13;
    public static final int SPINDEXER = 14;
    public static final int INTAKE_ROLLER_2 = 15;
    public static final int FEEDER = 18;

    // IDs 9, 16 and 17 are free.
    //
    // There is deliberately no climber entry: the climber mechanism never made it onto the
    // robot. The Climber subsystem that used to sit in rebuilt/subsystems has been removed
    // rather than left as finished-looking dead code. It is recoverable from git history if
    // the mechanism is ever built.

    private CanIds() {
    }
  }

  public static final class ShooterConstants {
    /** Closed-loop velocity gains for the flywheel leader. */
    public static final double kP = 0.00035;
    public static final double kI = 0.000001;
    public static final double kD = 0.0065;

    /** How close to the setpoint counts as ready to fire, in RPM. */
    public static final double RPM_TOLERANCE = 100.0;

    /** Largest trim, in RPM, the operator may dial in either direction. */
    public static final double MAX_OPERATOR_TRIM_RPM = 500.0;

    /** RPM step applied per operator D-pad press. */
    public static final double OPERATOR_TRIM_STEP_RPM = 10.0;

    /**
     * Whether to hold an idle speed between shots instead of stopping.
     *
     * <p>This replaces a hard-coded {@code true} field that made the alternative branch
     * unreachable.
     */
    public static final boolean IDLE_BETWEEN_SHOTS = true;

    private ShooterConstants() {
    }
  }

  public static final class IntakeConstants {
    /** Open-loop roller speed while intaking or ejecting. */
    public static final double ROLLER_SPEED = 0.75;

    /** Deploy arm target, in motor rotations, when fully deployed. */
    public static final double DEPLOY_POSITION_ROTATIONS = 10.0;

    /** Deploy arm target, in motor rotations, when stowed. */
    public static final double STOW_POSITION_ROTATIONS = 0.0;

    /**
     * Soft limits for the deploy arm, in motor rotations.
     *
     * <p>These are applied to the SPARK MAX itself so they hold even during open-loop
     * manual jogging. The equivalent checks previously existed only as commented-out code
     * in {@code periodic()}.
     */
    public static final float DEPLOY_REVERSE_SOFT_LIMIT = 0.0f;
    public static final float DEPLOY_FORWARD_SOFT_LIMIT = 11.0f;

    /** Open-loop speeds for manual deploy jogging. */
    public static final double MANUAL_DEPLOY_SPEED = 0.2;
    public static final double MANUAL_RETRACT_SPEED = -0.25;

    /**
     * Speed the deploy arm holds when not being commanded.
     *
     * <p>Deliberately a small negative bias, which keeps the arm loaded against its stow
     * stop instead of drifting. Named as a hold rather than a stop so the behaviour is
     * obvious at the call site.
     */
    public static final double DEPLOY_HOLD_SPEED = -0.03;

    /** Closed-loop position gain for the deploy arm. */
    public static final double DEPLOY_kP = 0.05;

    /** How close to the deploy target counts as arrived, in rotations. */
    public static final double DEPLOY_TOLERANCE_ROTATIONS = 0.5;

    private IntakeConstants() {
    }
  }

  public static final class FeederConstants {
    /** Open-loop feeder speed while loading into the flywheel. */
    public static final double FEEDER_SPEED = 1.0;

    /** Open-loop spindexer speed while indexing. */
    public static final double SPINDEXER_SPEED = 1.0;

    /**
     * Speed the spindexer holds when not actively indexing.
     *
     * <p>Deliberately non-zero: a slow crawl keeps game pieces settled against the feeder
     * rather than letting them wedge. Named as a hold rather than a stop so the behaviour is
     * obvious at the call site.
     */
    public static final double SPINDEXER_HOLD_SPEED = 0.1;

    private FeederConstants() {
    }
  }

}
