package frc.robot.integration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.CommonConstants.HIDConstants;
import frc.robot.common.components.diagnostics.ExpectationMonitor;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.rebuilt.components.FieldState;
import frc.robot.rebuilt.subsystems.Feeder;
import frc.robot.rebuilt.subsystems.Intake;
import frc.robot.rebuilt.subsystems.Shooter;
import frc.robot.rebuilt.subsystems.Shooter.ShooterPosition;
import frc.robot.rebuilt.subsystems.smart.ScoringLocationLookup;
import frc.robot.testutil.HalFixture;
import frc.robot.testutil.SharedSubsystems;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

/**
 * Integration tests that exercise several subsystems together through the real command
 * scheduler and a simulated driver station.
 *
 * <p>These run entirely in simulation as part of {@code ./gradlew test} — no robot required.
 * Where the unit tests check a method, these check a sequence: stick to module state, button
 * to flywheel, hub state to interlock.
 *
 * <p>Deliberately does not construct {@code RebuiltContainer}: its static initialiser calls
 * PathPlanner's {@code AutoBuilder}, which needs deploy-directory settings that do not exist
 * in a unit test JVM. The wiring it performs is reproduced here instead.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class TeleopIntegrationTest {

  private SwerveDriveSubsystem drive;
  private Shooter shooter;
  private Intake intake;
  private Feeder feeder;

  /** Xbox axis indices. */
  private static final int LEFT_X = 0;
  private static final int LEFT_Y = 1;
  private static final int RIGHT_X = 4;

  @BeforeAll
  void setUpClass() {
    HalFixture.initialize();
    drive = SharedSubsystems.drive();
    shooter = SharedSubsystems.shooter();
    intake = SharedSubsystems.intake();
    feeder = SharedSubsystems.feeder();
  }

  @BeforeEach
  void setUp() {
    HalFixture.enableTeleop(true);
    HalFixture.centreDriverSticks();
    HalFixture.resetSubsystem(drive);
    HalFixture.resetSubsystem(shooter);
    HalFixture.resetSubsystem(intake);
    HalFixture.resetSubsystem(feeder);

    SharedSubsystems.setHubActive(true);
    shooter.stopShooter();
    shooter.resetOperatorModifier();
    shooter.setCurrentShooterPosition(ShooterPosition.HUB);
    intake.stopRollers();
    feeder.stopLoad();
    feeder.stopCycle();

    drive.drive(0, 0, 0, false);
    drive.resetOdometry(new Pose2d());
  }

  @Test
  @DisplayName("Driver stick input reaches the swerve modules through the scheduler")
  void stickInputReachesModules() {
    // Wire the drivetrain exactly as RebuiltContainer does.
    drive.setDefaultCommand(drive.driveCommand(
        () -> -HIDConstants.DRIVER_CONTROLLER.getLeftY(),
        () -> -HIDConstants.DRIVER_CONTROLLER.getLeftX(),
        () -> -HIDConstants.DRIVER_CONTROLLER.getRightX(),
        true));

    HalFixture.centreDriverSticks();
    HalFixture.runScheduler(3);
    assertTrue(maxCommandedSpeed() < 1e-6, "Centred sticks must leave the robot still");

    // Push the left stick fully forward. On an Xbox controller forward is negative Y.
    HalFixture.setDriverAxis(LEFT_Y, -1.0);
    HalFixture.runScheduler(3);

    assertTrue(maxCommandedSpeed() > 1.0,
        "A full forward stick must command the modules. This is the end-to-end form of the "
            + "bug where driveCommand captured stick values once at construction.");

    HalFixture.centreDriverSticks();
    HalFixture.runScheduler(3);
    assertTrue(maxCommandedSpeed() < 1e-6, "Releasing the stick must stop the robot");
  }

  @Test
  @DisplayName("Strafe and rotation are independently commandable")
  void strafeAndRotateWork() {
    drive.setDefaultCommand(drive.driveCommand(
        () -> -HIDConstants.DRIVER_CONTROLLER.getLeftY(),
        () -> -HIDConstants.DRIVER_CONTROLLER.getLeftX(),
        () -> -HIDConstants.DRIVER_CONTROLLER.getRightX(),
        false));

    HalFixture.setDriverAxis(LEFT_X, 1.0);
    HalFixture.runScheduler(3);
    assertTrue(maxCommandedSpeed() > 1.0, "Strafe input must move the modules");

    HalFixture.centreDriverSticks();
    HalFixture.runScheduler(3);

    HalFixture.setDriverAxis(RIGHT_X, 1.0);
    HalFixture.runScheduler(3);
    assertTrue(maxCommandedSpeed() > 0.1, "Rotation input must move the modules");
  }

  @Test
  @DisplayName("Full shoot sequence: select preset, spin up, feed")
  void shootSequence() {
    SharedSubsystems.setHubActive(true);

    Command shoot = edu.wpi.first.wpilibj2.command.Commands
        .runOnce(() -> shooter.setCurrentShooterPosition(ShooterPosition.TRENCH))
        .alongWith(edu.wpi.first.wpilibj2.command.Commands.run(shooter::runShooter, shooter));

    CommandScheduler.getInstance().schedule(shoot);
    HalFixture.runScheduler(5);

    assertEquals(ShooterPosition.TRENCH, shooter.getCurrentShooterPosition());
    assertTrue(shooter.isRunning(), "Holding the trench button must spin the flywheel");
    assertEquals(ShooterPosition.TRENCH.rpm, shooter.getTargetRPM(), 1e-9);

    // Feed while the flywheel is up.
    CommandScheduler.getInstance().schedule(feeder.loadAndCycleCommand());
    HalFixture.runScheduler(3);
    assertTrue(feeder.isLoading(), "The feeder must run to actually fire");
    assertTrue(feeder.isCycling());

    shoot.cancel();
    CommandScheduler.getInstance().schedule(feeder.stopLoadAndCycleCommand());
    HalFixture.runScheduler(3);
    assertFalse(feeder.isLoading());
  }

  @Test
  @DisplayName("Hub interlock blocks the whole shoot sequence when the goal is closed")
  void interlockBlocksShootSequence() {
    SharedSubsystems.setHubActive(false);

    Command shoot = edu.wpi.first.wpilibj2.command.Commands
        .run(shooter::runShooter, shooter);
    CommandScheduler.getInstance().schedule(shoot);
    HalFixture.runScheduler(10);

    assertFalse(shooter.isRunning(),
        "With the hub closed the flywheel must stay down for the whole sequence");

    shoot.cancel();
  }

  @Test
  @DisplayName("Interlock releasing mid-sequence lets the flywheel spin up")
  void interlockReleaseAllowsSpinUp() {
    SharedSubsystems.setHubActive(false);

    Command shoot = edu.wpi.first.wpilibj2.command.Commands.run(shooter::runShooter, shooter);
    CommandScheduler.getInstance().schedule(shoot);
    HalFixture.runScheduler(5);
    assertFalse(shooter.isRunning());

    // The hub opens; because the interlock is a live supplier, the running command should
    // start commanding the flywheel without being rescheduled.
    SharedSubsystems.setHubActive(true);
    HalFixture.runScheduler(5);
    assertTrue(shooter.isRunning(),
        "The interlock must be re-evaluated every loop, not cached at schedule time");

    shoot.cancel();
  }

  @Test
  @DisplayName("Intake and feeder can run together, as the fire binding requires")
  void intakeAndFeederCoexist() {
    CommandScheduler.getInstance().schedule(intake.intakeCommand());
    CommandScheduler.getInstance().schedule(feeder.loadAndCycleCommand());
    HalFixture.runScheduler(3);

    assertTrue(intake.isRunning(), "Intake and feeder must be able to run simultaneously");
    assertTrue(feeder.isLoading());
  }

  @Test
  @DisplayName("Field state and assist lookup stay consistent through a loop")
  void fieldStateDrivesAssistLookup() {
    HalFixture.enableTeleop(true);
    HalFixture.setGameData("R");

    FieldState fieldState = new FieldState();
    fieldState.update();

    ScoringLocationLookup.buildScoringLocations();
    ScoringLocationLookup.setRedAlliance(fieldState.isAllianceRed());

    drive.resetOdometry(new Pose2d(13.0, 4.0, new edu.wpi.first.math.geometry.Rotation2d()));
    drive.periodic();

    Pose2d nearest = ScoringLocationLookup.findClosest(drive.getPose());
    assertTrue(Double.isFinite(nearest.getX()));
    assertEquals("hub", ScoringLocationLookup.findClosestName(drive.getPose()),
        "Sitting on the red hub, the nearest scoring location should be the hub");
  }

  @Test
  @DisplayName("Expectation monitor stays clean across a normal teleop sequence")
  void expectationsHoldDuringNormalOperation() {
    ExpectationMonitor monitor = ExpectationMonitor.getInstance();
    monitor.clear();

    monitor.register("PoseFinite", "Pose stays finite",
        () -> Double.isFinite(drive.getPose().getX()), 1);
    monitor.register("InterlockRespected", "Flywheel never runs with the hub closed",
        () -> shooter.isHubActive() || !shooter.isRunning(), 1);

    drive.setDefaultCommand(drive.driveCommand(
        () -> -HIDConstants.DRIVER_CONTROLLER.getLeftY(),
        () -> 0.0,
        () -> 0.0,
        true));

    HalFixture.setDriverAxis(LEFT_Y, -0.5);
    for (int i = 0; i < 20; i++) {
      HalFixture.runScheduler(1);
      drive.periodic();
      monitor.update();
    }

    assertTrue(monitor.allOk(),
        "A normal drive-and-shoot sequence must not trip any expectation. Broken: "
            + monitor.getViolations().stream()
                .map(ExpectationMonitor.Expectation::getName)
                .toList());

    monitor.clear();
  }

  @Test
  @DisplayName("Expectation monitor catches a drivetrain that ignores the sticks")
  void expectationsCatchFrozenDrivetrain() {
    ExpectationMonitor monitor = ExpectationMonitor.getInstance();
    monitor.clear();

    // Reproduce the original defect deliberately: a default command built from values
    // sampled once, rather than suppliers polled every loop.
    double frozenX = -HIDConstants.DRIVER_CONTROLLER.getLeftY(); // sampled with sticks centred
    drive.setDefaultCommand(drive.driveCommand(() -> frozenX, () -> 0.0, () -> 0.0, true));

    monitor.register(
        "DriveRespondsToStick",
        "When the driver commands motion, some module is commanded to move",
        () -> {
          double commanded = Math.abs(SwerveDriveSubsystem.applyDeadband(
              -HIDConstants.DRIVER_CONTROLLER.getLeftY()));
          if (commanded <= 0) {
            return true;
          }
          for (SwerveModuleState state : drive.getDesiredModuleStates()) {
            if (Math.abs(state.speedMetersPerSecond) > 1e-3) {
              return true;
            }
          }
          return false;
        },
        3);

    // Driver asks for full forward, but the frozen command keeps sending zero.
    HalFixture.setDriverAxis(LEFT_Y, -1.0);
    for (int i = 0; i < 10; i++) {
      HalFixture.runScheduler(1);
      monitor.update();
    }

    assertFalse(monitor.allOk(),
        "The monitor must notice a drivetrain that ignores the sticks — this is what makes "
            + "the original bug visible in a log instead of only in a driver's complaint");

    monitor.clear();
  }

  @Test
  @DisplayName("Fire-sequence composition is legal: no subsystem required twice in parallel")
  void fireSequenceCompositionIsLegal() {
    // WPILib throws IllegalArgumentException("Multiple commands in a parallel composition
    // cannot require the same subsystems") at construction time. Because the bindings are
    // built inside a static initialiser, that exception kills the whole robot program before
    // it ever reaches teleop — which is exactly what happened once the intake commands
    // gained proper subsystem requirements.
    //
    // This reproduces the container's composite shape so the failure surfaces in a test
    // rather than on the field.
    Command fire = edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
          feeder.load();
          feeder.cycle();
        }, feeder)
        .alongWith(edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
          intake.manualReverseDeploy();
          intake.intake();
        }, intake));

    Command release = edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
          feeder.stopLoad();
          feeder.holdCycle();
        }, feeder)
        .alongWith(edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
          intake.stopDeploy();
          intake.holdRollers();
        }, intake));

    assertTrue(fire.getRequirements().contains(feeder));
    assertTrue(fire.getRequirements().contains(intake));

    CommandScheduler.getInstance().schedule(fire);
    HalFixture.runScheduler(3);
    assertTrue(feeder.isLoading(), "The fire composition must actually run the feeder");
    assertTrue(intake.isRunning(), "The fire composition must actually run the intake");

    CommandScheduler.getInstance().schedule(release);
    HalFixture.runScheduler(3);
    assertFalse(feeder.isLoading());
    assertFalse(intake.isRunning());
  }

  @Test
  @DisplayName("Every intake and feeder command declares its subsystem requirement")
  void commandsDeclareRequirements() {
    // Without requirements, two commands touching the same mechanism run simultaneously and
    // fight instead of interrupting one another.
    assertTrue(intake.intakeCommand().getRequirements().contains(intake));
    assertTrue(intake.stopIntakeCommand().getRequirements().contains(intake));
    assertTrue(intake.deploy().getRequirements().contains(intake));
    assertTrue(feeder.loadAndCycleCommand().getRequirements().contains(feeder));
    assertTrue(feeder.stopLoadAndCycleCommand().getRequirements().contains(feeder));
  }

  /** @return the largest absolute commanded module speed, in m/s. */
  private double maxCommandedSpeed() {
    double max = 0;
    for (SwerveModuleState state : drive.getDesiredModuleStates()) {
      max = Math.max(max, Math.abs(state.speedMetersPerSecond));
    }
    return max;
  }
}
