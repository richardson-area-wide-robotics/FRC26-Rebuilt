package frc.robot.rebuilt.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.rebuilt.RebuiltConstants.ShooterConstants;
import frc.robot.rebuilt.subsystems.Shooter.ShooterPosition;
import frc.robot.testutil.HalFixture;
import frc.robot.testutil.SharedSubsystems;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

/**
 * Tests for the shooter: preset selection, the operator trim, and the hub interlock.
 *
 * <p>The interlock is injected as a {@code BooleanSupplier}, which is what makes these tests
 * possible at all — it previously read a mutable static off the container.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class ShooterTest {

  private Shooter shooter;

  @BeforeAll
  void setUpClass() {
    HalFixture.initialize();
    shooter = SharedSubsystems.shooter();
  }

  @BeforeEach
  void setUp() {
    HalFixture.enableTeleop(true);
    SharedSubsystems.setHubActive(true);
    shooter.stopShooter();
    shooter.resetOperatorModifier();
    shooter.setCurrentShooterPosition(ShooterPosition.HUB);
  }

  @Test
  @DisplayName("Target RPM follows the selected preset")
  void targetFollowsPreset() {
    shooter.setCurrentShooterPosition(ShooterPosition.CORNER);
    assertEquals(ShooterPosition.CORNER.rpm, shooter.getTargetRPM(), 1e-9);

    shooter.setCurrentShooterPosition(ShooterPosition.HUB);
    assertEquals(ShooterPosition.HUB.rpm, shooter.getTargetRPM(), 1e-9);
  }

  @Test
  @DisplayName("Operator trim adds to and subtracts from the preset")
  void trimAdjustsTarget() {
    double base = shooter.getTargetRPM();

    shooter.raiseOperatorModifer(10);
    assertEquals(base + 10, shooter.getTargetRPM(), 1e-9);

    shooter.lowerOperatorModifer(10);
    assertEquals(base, shooter.getTargetRPM(), 1e-9);

    shooter.lowerOperatorModifer(10);
    assertEquals(base - 10, shooter.getTargetRPM(), 1e-9);
  }

  @Test
  @DisplayName("Operator trim saturates instead of running away")
  void trimIsClamped() {
    // Previously unbounded: holding the D-pad could drive the commanded RPM arbitrarily
    // high, or negative.
    for (int i = 0; i < 500; i++) {
      shooter.raiseOperatorModifer(10);
    }
    assertEquals(ShooterConstants.MAX_OPERATOR_TRIM_RPM, shooter.getOperatorModifier(), 1e-9);

    for (int i = 0; i < 1000; i++) {
      shooter.lowerOperatorModifer(10);
    }
    assertEquals(-ShooterConstants.MAX_OPERATOR_TRIM_RPM, shooter.getOperatorModifier(), 1e-9);
  }

  @Test
  @DisplayName("Trim can never drive the commanded RPM to zero or below")
  void trimCannotInvertTheTarget() {
    shooter.setCurrentShooterPosition(ShooterPosition.IDLE);
    for (int i = 0; i < 1000; i++) {
      shooter.lowerOperatorModifer(10);
    }
    assertTrue(shooter.getTargetRPM() > 0,
        "Even fully trimmed down, the commanded RPM must stay positive; was "
            + shooter.getTargetRPM());
  }

  @Test
  @DisplayName("Trim resets to zero on demand")
  void trimResets() {
    shooter.raiseOperatorModifer(50);
    shooter.resetOperatorModifier();
    assertEquals(0.0, shooter.getOperatorModifier(), 1e-9);
  }

  @Test
  @DisplayName("Flywheel runs when the hub is open")
  void runsWhenHubOpen() {
    SharedSubsystems.setHubActive(true);
    shooter.runShooter();
    assertTrue(shooter.isRunning());
  }

  @Test
  @DisplayName("Flywheel refuses to run when the hub is closed")
  void refusesWhenHubClosed() {
    SharedSubsystems.setHubActive(false);
    shooter.runShooter();
    assertFalse(shooter.isRunning(),
        "The interlock must stop the flywheel spinning into a closed goal");
  }

  @Test
  @DisplayName("Interlock reflects the injected supplier, not a cached value")
  void interlockIsLive() {
    SharedSubsystems.setHubActive(false);
    assertFalse(shooter.isHubActive());

    SharedSubsystems.setHubActive(true);
    assertTrue(shooter.isHubActive(),
        "The interlock is read through a supplier, so it must update without reconstruction");
  }

  @Test
  @DisplayName("Stopping clears the running flag")
  void stopClearsRunning() {
    shooter.runShooter();
    shooter.stopShooter();
    assertFalse(shooter.isRunning());
  }

  @Test
  @DisplayName("Idle-or-stop drops to the idle preset while the hub is open")
  void idleOrStopIdlesWhenOpen() {
    SharedSubsystems.setHubActive(true);
    shooter.setCurrentShooterPosition(ShooterPosition.CORNER);

    shooter.idleOrStop();

    assertEquals(ShooterPosition.IDLE, shooter.getCurrentShooterPosition(),
        "Between shots the flywheel should hold the idle preset, not the last shot preset");
    assertTrue(shooter.isRunning());
  }

  @Test
  @DisplayName("Idle-or-stop stops entirely while the hub is closed")
  void idleOrStopStopsWhenClosed() {
    SharedSubsystems.setHubActive(false);
    shooter.idleOrStop();
    assertFalse(shooter.isRunning());
  }

  @Test
  @DisplayName("At-target is false while the flywheel is stopped")
  void notAtTargetWhenStopped() {
    shooter.stopShooter();
    assertFalse(shooter.isAtTargetRPM(),
        "A stopped flywheel must never report itself ready to fire");
  }

  @Test
  @DisplayName("Named auto commands exist for the shooter")
  void namedCommandsExist() {
    assertTrue(shooter.runShooterCommand() != null);
    assertTrue(shooter.stopShooterCommand() != null);
  }
}
