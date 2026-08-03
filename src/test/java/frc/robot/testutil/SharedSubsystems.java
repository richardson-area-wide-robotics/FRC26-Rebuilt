package frc.robot.testutil;

import java.util.function.BooleanSupplier;

import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.rebuilt.RebuiltConstants.CanIds;
import frc.robot.rebuilt.subsystems.Feeder;
import frc.robot.rebuilt.subsystems.Intake;
import frc.robot.rebuilt.subsystems.Shooter;

/**
 * Lazily-constructed, JVM-wide single instances of every hardware-backed subsystem.
 *
 * <p>REVLib refuses to construct two controller objects for the same CAN ID:
 * {@code IllegalStateException: A CANSparkMax instance has already been created with this
 * device ID}. Gradle runs the whole test suite in one JVM, so a subsystem may only be built
 * once for the entire run — not per test, and not per test class.
 *
 * <p>Tests therefore share these instances and reset the state they care about in
 * {@code @BeforeEach} rather than rebuilding hardware.
 */
public final class SharedSubsystems {

  private static SwerveDriveSubsystem drive;
  private static Shooter shooter;
  private static Intake intake;
  private static Feeder feeder;

  /** Controls the shooter's hub interlock in tests. Defaults to open. */
  private static boolean hubActive = true;

  private SharedSubsystems() {
  }

  public static synchronized SwerveDriveSubsystem drive() {
    HalFixture.initialize();
    if (drive == null) {
      drive = new SwerveDriveSubsystem();
    }
    return drive;
  }

  /**
   * The shared shooter. Its hub interlock reads {@link #setHubActive(boolean)}.
   *
   * @return the shared shooter instance.
   */
  public static synchronized Shooter shooter() {
    HalFixture.initialize();
    if (shooter == null) {
      BooleanSupplier interlock = () -> hubActive;
      shooter = new Shooter(CanIds.SHOOTER_LEADER, CanIds.SHOOTER_FOLLOWER, interlock);
    }
    return shooter;
  }

  public static synchronized Intake intake() {
    HalFixture.initialize();
    if (intake == null) {
      intake = new Intake(CanIds.INTAKE_ROLLER_1, CanIds.INTAKE_ROLLER_2, CanIds.INTAKE_DEPLOY);
    }
    return intake;
  }

  public static synchronized Feeder feeder() {
    HalFixture.initialize();
    if (feeder == null) {
      feeder = new Feeder(CanIds.FEEDER, CanIds.SPINDEXER);
    }
    return feeder;
  }

  /**
   * Opens or closes the shooter's hub interlock for the shared shooter.
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
