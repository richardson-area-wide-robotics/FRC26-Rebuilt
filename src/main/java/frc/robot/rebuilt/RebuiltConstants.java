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
   * <p><b>Motor inventory.</b> Recorded here because not knowing which motor sits where is exactly
   * what let a wrong free-speed constant survive review.
   *
   * <p><b>The controller determines the motor on this robot.</b> Stated by the team as a complete
   * rule, and every mechanism below follows from it:
   *
   * <ul>
   *   <li><b>SPARK Flex &rarr; NEO Vortex</b>, always. Nothing else is ever connected to one.
   *   <li><b>SPARK MAX &rarr; NEO 2.0</b>, <em>except</em> a swerve steering motor, which is a
   *       NEO 550.
   * </ul>
   *
   * <p>Controller types are read straight from the code and are certain, so the motor column is
   * derived rather than guessed. Nothing here is marked CONFIRM any more.
   *
   * <pre>
   *   CAN 1,3,5,7   swerve drive      NEO Vortex   + SPARK Flex   6784 RPM free
   *   CAN 2,4,6,8   swerve steering   NEO 550      + SPARK MAX   11000 RPM free
   *                 (the steering exception to the SPARK MAX rule) + Through Bore V2 absolute
   *   CAN 10,11     shooter           NEO Vortex   + SPARK Flex   6784 RPM free
   *                 (independently confirmed: MotorKind.NEO_VORTEX in Shooter.java)
   *   CAN 13,15     intake rollers    NEO Vortex   + SPARK Flex   6784 RPM free
   *   CAN 18        feeder            NEO Vortex   + SPARK Flex   6784 RPM free
   *                 (aka "tower motor" — same mechanism; no separate tower exists)
   *   CAN 12        intake deploy     NEO 2.0      + SPARK MAX    5676 RPM free
   *   CAN 14        spindexer         NEO 2.0      + SPARK MAX    5676 RPM free
   * </pre>
   *
   * <p>The feeder was at one point recorded as a NEO 2.0 and carried as an open conflict, since it
   * is a SPARK Flex. The rule resolves it to a <b>Vortex</b>. Worth remembering that the two
   * candidates were 5676 and 6784 RPM — 19.5% apart, the same magnitude and the same failure mode
   * as the drive free-speed bug this inventory exists to prevent.
   *
   * <p><b>The NEO 2.0s on this robot are the intake deploy and the spindexer.</b> That is what makes
   * 5676 RPM a real figure for a real motor here, and it is why the wrong drive constant survived
   * review: it named a motor that exists on the robot, just not on the drive shaft.
   *
   * <p>The shop session still confirms rather than assumes: the load calibration measures unloaded
   * speed directly, so each mechanism's {@code FreeSpeed} entry under {@code LoadCalibration} will
   * show roughly 6,500 for the Vortex mechanisms and roughly 5,400 for the spindexer. A mechanism
   * landing near the wrong figure means the rule has an exception nobody has mentioned.
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

  /**
   * Where the robot's mechanisms point, relative to the chassis forward axis.
   *
   * <p><b>The intake and the shooter are 90 degrees apart on this robot, and the camera is in line
   * with the shooter.</b> That single fact has consequences the code cannot infer, so it lives here.
   *
   * <p>The chassis forward axis (+x) is whatever the robot drives toward on a forward stick. Taking
   * that as the <b>intake</b> direction, which is the usual convention for a robot with a floor
   * intake and is what {@code BUMP_REVERSE} assumes when it leads with the back, the shooter and the
   * camera both sit at 90 degrees to it.
   *
   * <p><b>This is what {@code AIM_AT_HUB} has to account for.</b> Aiming the chassis +x axis at the
   * hub points the <em>intake</em> at the hub and the shooter at whatever is 90 degrees away, so the
   * shot misses the field. The heading target has to be the bearing to the hub <em>minus</em> this
   * offset, so that once the robot is there the shooter is the thing pointing at the goal.
   */
  public static final class GeometryConstants {

    /**
     * CONFIRM THE SIGN — degrees from chassis forward (+x) to the direction the shooter fires.
     *
     * <p>Magnitude is 90, which is known. The sign is not, and it cannot be read off the code:
     *
     * <ul>
     *   <li>{@code +90} means the shooter fires out of the robot's <b>left</b> side, since +y is
     *       left in WPILib's frame.
     *   <li>{@code -90} means it fires out of the <b>right</b> side.
     * </ul>
     *
     * <p>To settle it: stand behind the robot looking the way it drives forward. If the shooter is on
     * your left, this is {@code +90}. Getting it backwards aims the robot 180 degrees away from the
     * hub, which at least fails obviously rather than subtly.
     */
    public static final double SHOOTER_YAW_OFFSET_DEGREES = 90.0;

    /**
     * Degrees from chassis forward to the camera's optical axis.
     *
     * <p>Deliberately defined as equal to {@link #SHOOTER_YAW_OFFSET_DEGREES} rather than as its own
     * number, because the camera is physically in line with the shooter — so they cannot disagree
     * without someone having moved the camera. {@code VisionConstants.ROBOT_TO_CAMERA} must carry
     * this same yaw, and a test asserts it does.
     *
     * <p>That matters: the placeholder transform has a yaw of 0, which would rotate every
     * tag-derived pose by 90 degrees. Vision would be confidently, enormously wrong.
     */
    public static final double CAMERA_YAW_OFFSET_DEGREES = SHOOTER_YAW_OFFSET_DEGREES;

    private GeometryConstants() {
    }
  }

  /**
   * Gear and pulley reductions, from CAD or a tooth count on the robot.
   *
   * <p>These are the numbers that turn a motor into a mechanism, and they are the one category where
   * <b>CAD is authoritative and a tape measure is useless</b> — you cannot measure a reduction, you
   * count teeth. Everything here should come off the CAD or the BOM.
   *
   * <p>Two of them are load-bearing rather than documentation: the drive reduction feeds the
   * feedforward, and the deploy reduction is what turns opaque motor rotations into arm degrees.
   */
  public static final class MechanismRatios {

    /**
     * CONFIRM FROM CAD — teeth on the MAXSwerve driving pinion.
     *
     * <p>REV ships 12T, 13T and 14T. The code assumes <b>14T</b>, and this is the single highest-risk
     * unconfirmed number left on the robot, because it scales the drive reduction and therefore the
     * feedforward and the top speed:
     *
     * <pre>
     *   12T -> reduction 5.500, free speed 4.92 m/s
     *   13T -> reduction 5.077, free speed 5.33 m/s
     *   14T -> reduction 4.714, free speed 5.74 m/s   (assumed)
     * </pre>
     *
     * <p>That is a 17% spread, which is the same order as the wrong-motor free speed bug and would
     * fail in exactly the same way: plausible numbers, wrong feedforward, closed loop fighting it.
     * <b>Count the teeth.</b> It lives in {@code CommonConstants.ModuleConstants} as
     * {@code kDrivingMotorPinionTeeth} and a test pins the two together.
     */
    public static final int DRIVE_PINION_TEETH = 14;

    /**
     * FROM CAD — total reduction from the intake deploy motor to the arm, motor turns per arm turn.
     *
     * <p>Set this and {@code IntakeConstants} stops speaking in opaque motor rotations. The deploy
     * target of 10 rotations and the soft limits of 0 to 11 currently mean nothing to a human, and
     * "whether they match real travel" has been on the unverified list since the first review. With a
     * reduction they convert to arm degrees and can simply be checked against the CAD travel.
     *
     * <p>Left at 1.0 until read off CAD, which makes {@link #deployRotationsToDegrees} the identity
     * and changes no behaviour — the conversion is offered, not imposed.
     */
    public static final double INTAKE_DEPLOY_REDUCTION = 1.0;

    /**
     * FROM CAD — reduction from shooter motor to flywheel. Greater than 1 means geared down.
     *
     * <p>Does not affect the closed loop, which runs on the motor encoder, so the RPM presets in
     * {@link ShooterConstants} are motor RPM either way and the range model is empirical. What it does
     * give is a sanity check: motor RPM, this reduction and the flywheel diameter together predict the
     * ball's exit speed, and if that comes out implausible for the shot distances the team actually
     * makes then one of the three is wrong.
     */
    public static final double SHOOTER_REDUCTION = 1.0;

    /** FROM CAD — flywheel diameter in metres. Used only for the exit-speed sanity check. */
    public static final double SHOOTER_FLYWHEEL_DIAMETER_METERS = 0.1016;

    /**
     * @param motorRotations Deploy motor rotations.
     * @return arm travel in degrees.
     */
    public static double deployRotationsToDegrees(double motorRotations) {
      return motorRotations / INTAKE_DEPLOY_REDUCTION * 360.0;
    }

    /**
     * @param armDegrees Desired arm travel in degrees.
     * @return the motor rotations that produces it.
     */
    public static double deployDegreesToRotations(double armDegrees) {
      return armDegrees / 360.0 * INTAKE_DEPLOY_REDUCTION;
    }

    /**
     * @param motorRpm Shooter motor RPM.
     * @return the ball's approximate exit speed in m/s.
     *
     *     <p>Surface speed of the flywheel, halved. A ball squeezed between a flywheel and a static
     *     hood leaves at roughly half the wheel's surface speed, because the contact patch has the
     *     wheel moving on one side and nothing moving on the other. Rough, but enough to tell a
     *     plausible shooter from an impossible one.
     */
    public static double approximateExitSpeed(double motorRpm) {
      double flywheelRps = motorRpm / SHOOTER_REDUCTION / 60.0;
      return flywheelRps * Math.PI * SHOOTER_FLYWHEEL_DIAMETER_METERS / 2.0;
    }

    private MechanismRatios() {
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

    /** Free speed of a NEO Vortex, RPM. Intake rollers, feeder and shooter. */
    private static final double VORTEX_FREE_RPM = 6784.0;

    /** Free speed of a NEO 2.0, RPM. Intake deploy and spindexer. */
    private static final double NEO_2_0_FREE_RPM = 5676.0;

    /**
     * Fraction of free speed a mechanism actually reaches unloaded.
     *
     * <p>Gearing and belt drag, bearing friction, air resistance and bus voltage all take a cut, so
     * nothing reaches its datasheet figure. 0.90 is a reasonable opening bid.
     *
     * <p><b>Erring low is the safe direction and that is why this is 0.90 rather than 0.95.</b> The
     * jam threshold is a fraction of expected speed, so too low means jams take marginally longer to
     * catch, while too high means a healthy mechanism reads as permanently slow and the robot starts
     * clearing a jam that does not exist mid-match.
     *
     * <p>{@code MotorLoadMonitor} learns the real ratio at runtime anyway, so this only has to be
     * close enough to behave sensibly before it has learned anything.
     */
    private static final double UNLOADED_SPEED_FRACTION = 0.90;
    /** MEASURE — amps above idle indicating a piece is moving through the intake rollers. */
    public static final double INTAKE_WORK_EXCESS_AMPS = 10.0;

    /**
     * Expected unloaded intake roller speed, in motor RPM. <b>Derived, not guessed.</b>
     *
     * <p>NEO Vortex on a SPARK Flex, commanded at {@code ROLLER_SPEED}. Duty cycle scales speed
     * roughly linearly, so free speed times demand times the drag allowance is the estimate. Works
     * out near 4,580.
     *
     * <p>Derived rather than typed so that changing {@code ROLLER_SPEED} cannot silently leave the
     * jam threshold calibrated for the old demand — which is exactly the kind of coupling that
     * survives review because both numbers look individually reasonable.
     */
    public static final double INTAKE_EXPECTED_RPM =
        VORTEX_FREE_RPM * IntakeConstants.ROLLER_SPEED * UNLOADED_SPEED_FRACTION;

    /** MEASURE — amps above idle indicating a piece moving through the spindexer. */
    public static final double SPINDEXER_WORK_EXCESS_AMPS = 8.0;

    /**
     * Expected unloaded spindexer speed, in motor RPM. <b>Derived, not guessed.</b>
     *
     * <p>NEO 2.0 on a SPARK MAX — one of only two on the robot, the other being the intake deploy —
     * commanded at {@code SPINDEXER_SPEED}. Works out near 5,110.
     */
    public static final double SPINDEXER_EXPECTED_RPM =
        NEO_2_0_FREE_RPM * FeederConstants.SPINDEXER_SPEED * UNLOADED_SPEED_FRACTION;

    /** MEASURE — amps above idle indicating a piece moving through the feeder. */
    public static final double FEEDER_WORK_EXCESS_AMPS = 10.0;

    /**
     * Expected unloaded feeder speed, in motor RPM. <b>Derived, not guessed.</b>
     *
     * <p>NEO Vortex on a SPARK Flex, commanded at {@code FEEDER_SPEED}. Works out near 6,110 — a
     * long way from the flat 5,000 this used to carry, which was 26% low and would have put the jam
     * threshold correspondingly low.
     */
    public static final double FEEDER_EXPECTED_RPM =
        VORTEX_FREE_RPM * FeederConstants.FEEDER_SPEED * UNLOADED_SPEED_FRACTION;

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
