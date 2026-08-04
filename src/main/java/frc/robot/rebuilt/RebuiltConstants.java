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

  /**
   * CAN IDs for the 2026 superstructure. Swerve IDs live in {@code CommonConstants}.
   *
   * <p><b>Motor inventory.</b> Recorded here because not knowing which motor sits where is
   * exactly what let a wrong free-speed constant survive review: 5676 RPM is a real figure for
   * a motor that <em>is</em> on this robot, just not on the drive shaft.
   *
   * <pre>
   *   CAN 1,3,5,7   swerve drive      NEO Vortex   + SPARK Flex   6784 RPM free
   *   CAN 2,4,6,8   swerve steering   NEO 550      + SPARK MAX    + Through Bore V2 absolute
   *   CAN 10,11     shooter           NEO Vortex   + SPARK Flex   (MotorKind.NEO_VORTEX in code)
   *   CAN 13,15     intake rollers    SPARK Flex                  CONFIRM motor
   *   CAN 18        feeder            SPARK Flex                  CONFIRM motor
   *   CAN 12        intake deploy     SPARK MAX                   CONFIRM motor
   *   CAN 14        spindexer         SPARK MAX                   CONFIRM motor
   * </pre>
   *
   * <p>The controller types above are read straight from the code and are certain. The
   * superstructure <em>motors</em> are a mix of NEO Vortex, NEO 2.0 and NEO 550 — fill in the
   * four marked CONFIRM rather than assuming, since only free speed and current limits depend
   * on it and both matter.
   *
   * <p>Free speeds for reference: NEO Vortex 6784 RPM, NEO 2.0 and NEO 1.1 both 5676 RPM.
   */
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

  /**
   * Thresholds for detecting game pieces and jams from motor current.
   *
   * <p><strong>Every value here is reasoned, not measured.</strong> The robot already logs current
   * for each mechanism, so one session gives the real numbers: run each mechanism empty and note
   * {@code Load/<mechanism>/BaselineAmps}, then send a game piece through and note
   * {@code ExcessAmps}. Set the work threshold between the two, nearer the empty end.
   *
   * <p>Getting these wrong is not dangerous but it is annoying in both directions: too high and
   * pieces go uncounted, too low and the robot decides it is jammed during normal operation and
   * starts jostling itself mid-match.
   */
  public static final class LoadConstants {
    /** MEASURE — amps above idle indicating a piece is moving through the intake rollers. */
    public static final double INTAKE_WORK_EXCESS_AMPS = 10.0;

    /** MEASURE — unloaded intake roller speed, in motor RPM. */
    public static final double INTAKE_EXPECTED_RPM = 5000.0;

    /** MEASURE — amps above idle indicating a piece moving through the spindexer. */
    public static final double SPINDEXER_WORK_EXCESS_AMPS = 8.0;

    /** MEASURE — unloaded spindexer speed, in motor RPM. */
    public static final double SPINDEXER_EXPECTED_RPM = 5000.0;

    /** MEASURE — amps above idle indicating a piece moving through the feeder. */
    public static final double FEEDER_WORK_EXCESS_AMPS = 10.0;

    /** MEASURE — unloaded feeder speed, in motor RPM. */
    public static final double FEEDER_EXPECTED_RPM = 5000.0;

    /**
     * MEASURE — amps above idle indicating a game piece is passing through the flywheel.
     *
     * <p>Higher than the other mechanisms because a flywheel at speed carries real momentum: a piece
     * entering takes a large bite out of it and the closed loop answers with a lot of current.
     *
     * <p>There is deliberately no {@code SHOOTER_EXPECTED_RPM}. The flywheel's expected speed is its
     * live setpoint, which ranges from 1700 to 4500 RPM, so it is supplied to the monitor as a
     * supplier instead of pinned here.
     */
    public static final double SHOOTER_WORK_EXCESS_AMPS = 20.0;

    /**
     * Fraction of setpoint below which the flywheel counts as jammed.
     *
     * <p>Much tighter than the roller figure. A flywheel is speed-controlled and recovers within a
     * few hundred milliseconds of a shot, so it should never sit far below setpoint for long. 0.70
     * means "still 30% down well after the shot should have cleared", which on a flywheel means
     * something is rubbing or wedged rather than being shot.
     */
    public static final double SHOOTER_JAM_SPEED_FRACTION = 0.70;

    /**
     * MEASURE — loops to ignore further shot detections after counting one.
     *
     * <p>Shorter than {@link #PIECE_REFRACTORY_LOOPS} because the flywheel is the fastest point in
     * the path: the feeder can push pieces through back to back, and a refractory period longer than
     * the gap between them would undercount a volley. 12 loops is 240 ms.
     */
    public static final int SHOT_REFRACTORY_LOOPS = 12;

    /**
     * Fraction of expected speed below which a mechanism counts as stuck.
     *
     * <p>0.30 is deliberately generous: a roller genuinely slows when it bites a piece, and calling
     * that a jam would trigger jostling every time the robot did its job.
     */
    public static final double JAM_SPEED_FRACTION = 0.30;

    /**
     * Consecutive loops of high current and low speed before declaring a jam.
     *
     * <p>15 loops is 300 ms. Long enough that a piece seating does not trigger it, short enough that
     * a real jam is cleared before the driver has finished noticing.
     */
    public static final int JAM_CONFIRM_LOOPS = 15;

    /** Loops of sustained load before a game piece is counted. 3 loops is 60 ms. */
    public static final int PIECE_SUSTAIN_LOOPS = 3;

    /**
     * MEASURE — loops to ignore further detections after counting one piece.
     *
     * <p>Set from the fastest the mechanism can physically pass two pieces. 25 loops is half a
     * second; if pieces can arrive faster than that, lower it or the second will be missed.
     */
    public static final int PIECE_REFRACTORY_LOOPS = 25;

    private LoadConstants() {
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
