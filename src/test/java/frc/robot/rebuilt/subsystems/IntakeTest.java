package frc.robot.rebuilt.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.rebuilt.RebuiltConstants.IntakeConstants;
import frc.robot.testutil.HalFixture;
import frc.robot.common.components.diagnostics.HardStopDetector;
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

  @org.junit.jupiter.api.Nested
  @DisplayName("Profiled deploy control")
  class ProfiledDeploy {

    @Test
    @DisplayName("periodic() never enters profiled mode on its own")
    void periodicDoesNotStartProfiling() {
      // The property that actually matters, and it is order-independent — unlike asserting the
      // initial mode, which fails once another test in this class has set a goal on the shared
      // instance. What must hold is that only an explicit goal starts profiled control: if periodic()
      // could enter it, the arm would drive toward a goal nobody set the moment the robot enabled.
      intake.stopDeploy();
      assertEquals(Intake.DeployMode.MANUAL, intake.getDeployMode());

      for (int i = 0; i < 5; i++) {
        intake.periodic();
      }

      assertEquals(Intake.DeployMode.MANUAL, intake.getDeployMode(),
          "profiled control must only ever be entered by asking for a goal");
    }

    @Test
    @DisplayName("Asking for a goal switches to profiled control")
    void goalEntersProfiledMode() {
      intake.deployToGoal(5.0);
      assertEquals(Intake.DeployMode.PROFILED, intake.getDeployMode());
    }

    @Test
    @DisplayName("Every manual path hands control back, so nothing fights the profile")
    void manualPathsLeaveProfiledMode() {
      // Three ways the arm gets driven directly: an operator jog either way, and the idle hold. If
      // any of them left the mode at PROFILED, periodic() would keep writing profile voltage over
      // the top of the manual command every loop.
      for (Runnable manual : new Runnable[] {
          intake::manualDeploy, intake::manualReverseDeploy, intake::stopDeploy}) {
        intake.deployToGoal(5.0);
        assertEquals(Intake.DeployMode.PROFILED, intake.getDeployMode());

        manual.run();
        assertEquals(Intake.DeployMode.MANUAL, intake.getDeployMode(),
            "a manual command must take the arm out of profiled mode");
      }
    }

    @Test
    @DisplayName("The deploy and stow commands both use the profile")
    void commandsUseTheProfile() {
      // These used to call RobotUtils.moveToPosition, which is plain kPosition on the SPARK: output
      // proportional to error, so a full-travel move started at maximum output and slammed.
      intake.stopDeploy();
      intake.deploy().initialize();
      assertEquals(Intake.DeployMode.PROFILED, intake.getDeployMode());

      intake.stopDeploy();
      intake.reverseDeploy().initialize();
      assertEquals(Intake.DeployMode.PROFILED, intake.getDeployMode());
    }

    @Test
    @DisplayName("A goal beyond a learned hard stop is clamped to the stop")
    void goalClampsToLearnedStops() {
      // Teach the detector where the stops are, the way the real arm would: pushing, frozen, current
      // high, for longer than the sustain window.
      HardStopDetector stops = intake.getDeployStops();
      stops.resetLearned();
      for (int i = 0; i < 40; i++) {
        stops.update(0.0, 20.0, -0.25);
      }
      for (int i = 0; i < 40; i++) {
        stops.update(9.0, 20.0, 0.20);
      }

      assertEquals(9.0, stops.getLearnedStop(HardStopDetector.End.HIGH), 0.05);

      // Ask for more travel than exists. Without clamping the profile drives into steel and holds
      // there, which is current draw for the rest of the match.
      intake.deployToGoal(15.0);
      assertTrue(intake.getDeployGoal() <= 9.05,
          "goal should have been clamped to the learned stop, got " + intake.getDeployGoal());

      // And a goal inside the travel is left alone.
      intake.deployToGoal(4.0);
      assertEquals(4.0, intake.getDeployGoal(), 1e-6);
    }

    @Test
    @DisplayName("With no learned stops the goal is passed through unchanged")
    void unclampedWithoutLearnedStops() {
      // Clamping must not invent limits before anything has been measured, or the arm would be
      // restricted by a guess rather than by the mechanism.
      intake.getDeployStops().resetLearned();
      intake.deployToGoal(7.5);
      assertEquals(7.5, intake.getDeployGoal(), 1e-6);
    }
  }
}
