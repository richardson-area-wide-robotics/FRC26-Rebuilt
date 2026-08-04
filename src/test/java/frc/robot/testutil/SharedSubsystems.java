package frc.robot.testutil;

import java.util.function.BooleanSupplier;

import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.rebuilt.RebuiltContainer;
import frc.robot.rebuilt.subsystems.Feeder;
import frc.robot.rebuilt.subsystems.Intake;
import frc.robot.rebuilt.subsystems.Shooter;

/**
 * Lazily-constructed, JVM-wide single instances of every hardware-backed subsystem.
 *
 * <p>REVLib refuses to construct two controller objects for the same CAN ID:
 * {@code IllegalStateException: A CANSparkMax instance has already been created with this device
 * ID}. Gradle runs the whole suite in one JVM, so a given CAN ID may only be claimed once for the
 * entire run — not per test, and not per test class.
 *
 * <p>That constraint shapes this class in two ways:
 *
 * <ul>
 *   <li><b>The drivetrain is shared with production.</b> {@link #drive()} returns
 *       {@code RebuiltContainer.DRIVE_SUBSYSTEM} rather than building a second one, because the
 *       swerve CAN IDs are fixed in {@code DriveConstants} and cannot be varied. A bonus: the
 *       drivetrain tests then exercise the same object the robot actually uses.</li>
 *   <li><b>The superstructure uses test-only CAN IDs.</b> Loading {@code RebuiltContainer} claims
 *       10, 11, 12, 13, 14, 15 and 18 for its own shooter, intake and feeder, so the instances
 *       here take IDs in the 40s. Nothing is attached to them, which does not matter in
 *       simulation — REVLib is content to talk to a device that never answers — and it lets tests
 *       control a shooter's hub interlock without reaching into production field state.</li>
 * </ul>
 */
public final class SharedSubsystems {

  /**
   * CAN IDs used only by tests.
   *
   * <p>Deliberately far from anything real so a collision is obvious rather than subtle. If these
   * ever clash with hardware the symptom is an {@code IllegalStateException} at construction, not
   * silent misbehaviour.
   */
  private static final int TEST_SHOOTER_LEADER = 40;
  private static final int TEST_SHOOTER_FOLLOWER = 41;
  private static final int TEST_INTAKE_ROLLER_1 = 42;
  private static final int TEST_INTAKE_ROLLER_2 = 43;
  private static final int TEST_INTAKE_DEPLOY = 44;
  private static final int TEST_FEEDER = 45;
  private static final int TEST_SPINDEXER = 46;

  private static Shooter shooter;
  private static Intake intake;
  private static Feeder feeder;

  /** Controls the shooter's hub interlock in tests. Defaults to open. */
  private static boolean hubActive = true;

  private SharedSubsystems() {
  }

  /**
   * The production drivetrain.
   *
   * <p>Returns {@code RebuiltContainer.DRIVE_SUBSYSTEM} because the swerve CAN IDs are fixed and a
   * second instance would collide. Loading that class also constructs the production
   * superstructure and vision subsystems, which is harmless in simulation.
   *
   * @return the shared drivetrain.
   */
  public static synchronized SwerveDriveSubsystem drive() {
    HalFixture.initialize();
    return RebuiltContainer.DRIVE_SUBSYSTEM;
  }

  /**
   * A test-only shooter whose hub interlock is controlled by {@link #setHubActive(boolean)}.
   *
   * @return the shared test shooter.
   */
  public static synchronized Shooter shooter() {
    HalFixture.initialize();
    if (shooter == null) {
      BooleanSupplier interlock = () -> hubActive;
      shooter = new Shooter(TEST_SHOOTER_LEADER, TEST_SHOOTER_FOLLOWER, interlock);
    }
    return shooter;
  }

  /** @return the shared test intake. */
  public static synchronized Intake intake() {
    HalFixture.initialize();
    if (intake == null) {
      intake = new Intake(TEST_INTAKE_ROLLER_1, TEST_INTAKE_ROLLER_2, TEST_INTAKE_DEPLOY);
    }
    return intake;
  }

  /** @return the shared test feeder. */
  public static synchronized Feeder feeder() {
    HalFixture.initialize();
    if (feeder == null) {
      feeder = new Feeder(TEST_FEEDER, TEST_SPINDEXER);
    }
    return feeder;
  }

  /**
   * Opens or closes the shooter's hub interlock for the shared test shooter.
   *
   * @param active true to allow the flywheel to spin.
   */
  public static void setHubActive(boolean active) {
    hubActive = active;
  }

  /** @return whether the test interlock currently reports the hub as open. */
  public static boolean isHubActive() {
    return hubActive;
  }
}
