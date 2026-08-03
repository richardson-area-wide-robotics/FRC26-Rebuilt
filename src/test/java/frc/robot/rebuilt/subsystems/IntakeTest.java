package frc.robot.rebuilt.subsystems;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.rebuilt.RebuiltConstants.IntakeConstants;
import frc.robot.testutil.HalFixture;
import frc.robot.testutil.SharedSubsystems;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

/**
 * Tests for the intake: roller state, the hold-versus-stop distinction, and deploy limits.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class IntakeTest {

  private Intake intake;

  @BeforeAll
  void setUpClass() {
    HalFixture.initialize();
    intake = SharedSubsystems.intake();
  }

  @BeforeEach
  void setUp() {
    HalFixture.enableTeleop(true);
    intake.stopRollers();
  }

  @Test
  @DisplayName("Intaking marks the rollers as running")
  void intakingSetsRunning() {
    intake.intake();
    assertTrue(intake.isRunning());
  }

  @Test
  @DisplayName("Ejecting marks the rollers as running")
  void outtakingSetsRunning() {
    intake.outtake();
    assertTrue(intake.isRunning());
  }

  @Test
  @DisplayName("Stopping the rollers clears the running flag")
  void stopClearsRunning() {
    intake.intake();
    intake.stopRollers();
    assertFalse(intake.isRunning());
  }

  @Test
  @DisplayName("Holding the rollers is not the same as running them")
  void holdIsNotRunning() {
    intake.intake();
    intake.holdRollers();
    assertFalse(intake.isRunning(),
        "A retention bias is not intaking. The original stop() applied this bias while "
            + "claiming to have stopped.");
  }

  @Test
  @DisplayName("Deploy position is readable and finite")
  void deployPositionIsReadable() {
    assertTrue(Double.isFinite(intake.getDeployPosition()));
  }

  @Test
  @DisplayName("Encoder starts zeroed, so the arm reads as stowed")
  void startsStowed() {
    // The constructor zeroes the encoder, and stow is defined as position 0.
    assertTrue(intake.isStowed(),
        "With the encoder zeroed at construction the arm should read as stowed");
    assertFalse(intake.isDeployed());
  }

  @Test
  @DisplayName("Soft limits leave room for both travel extremes")
  void softLimitsAreCoherent() {
    assertTrue(IntakeConstants.DEPLOY_FORWARD_SOFT_LIMIT
            > IntakeConstants.DEPLOY_REVERSE_SOFT_LIMIT,
        "Forward soft limit must exceed the reverse limit");
    assertTrue(IntakeConstants.DEPLOY_FORWARD_SOFT_LIMIT
            >= IntakeConstants.DEPLOY_POSITION_ROTATIONS,
        "The deploy target must be reachable within the soft limits");
  }

  @Test
  @DisplayName("Manual jog speeds are opposite in sign")
  void manualSpeedsAreOpposite() {
    assertTrue(IntakeConstants.MANUAL_DEPLOY_SPEED > 0);
    assertTrue(IntakeConstants.MANUAL_RETRACT_SPEED < 0);
  }

  @Test
  @DisplayName("Deploy hold is a small bias, not a full command")
  void holdIsSmall() {
    assertTrue(Math.abs(IntakeConstants.DEPLOY_HOLD_SPEED) < 0.1,
        "A hold bias large enough to move the arm is not a hold");
  }

  @Test
  @DisplayName("Every intake command is constructible and requires the subsystem")
  void commandsAreWellFormed() {
    assertNotNull(intake.intakeCommand());
    assertNotNull(intake.stopIntakeCommand());
    assertNotNull(intake.deploy());
    assertNotNull(intake.reverseDeploy());
    assertNotNull(intake.manualDeployCommand());
    assertNotNull(intake.manualReverseDeployCommand());
    assertNotNull(intake.jiggleItALittleCommand());

    assertTrue(intake.intakeCommand().getRequirements().contains(intake),
        "Commands that move the rollers must require the subsystem so they can interrupt "
            + "each other rather than fighting");
  }
}
