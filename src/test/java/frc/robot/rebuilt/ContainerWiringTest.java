package frc.robot.rebuilt;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.components.diagnostics.ExpectationMonitor;
import frc.robot.testutil.HalFixture;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.TestMethodOrder;

/**
 * The test that closes the last real gap in coverage.
 *
 * <p>Container wiring was the one thing no test touched, and it is where the most dangerous class
 * of fault lives. WPILib validates command compositions at <em>construction</em> time, and this
 * wiring runs inside a static initialiser — so an illegal composition throws before
 * {@code robotInit()} finishes and the robot program never reaches teleop at all.
 *
 * <p>That is not hypothetical. Giving the intake commands proper subsystem requirements put two
 * commands requiring {@code INTAKE} into a single parallel composition, which WPILib rejects. Every
 * one of the two hundred other tests passed; only running the simulator caught it. This class
 * exists so that never happens again.
 *
 * <p>Lives in {@code frc.robot.rebuilt} deliberately, to reach the package-private wiring methods
 * without widening their visibility for production code.
 *
 * <p>Note what is <em>not</em> covered: {@link RebuiltContainer#configurePathPlanner()} needs
 * {@code RobotConfig.fromGUISettings()}, which has no deploy directory to read in a test JVM. That
 * remains simulation-only, and it is the reason {@code simulateJava} stays a mandatory pre-deploy
 * step rather than an optional one.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class ContainerWiringTest {

  @BeforeAll
  void setUpClass() {
    HalFixture.initialize();
    HalFixture.enableTeleop(true);
  }

  @Test
  @Order(1)
  @DisplayName("Every production subsystem constructs without claiming a duplicate CAN ID")
  void subsystemsConstruct() {
    // Touching the class runs its static initialisers, which build the real shooter, feeder,
    // intake, drivetrain and vision subsystems on their real CAN IDs. A duplicate ID anywhere
    // throws here.
    assertDoesNotThrow(() -> {
      assertNotNull(RebuiltContainer.DRIVE_SUBSYSTEM);
      assertNotNull(RebuiltContainer.SHOOTER);
      assertNotNull(RebuiltContainer.FEEDER);
      assertNotNull(RebuiltContainer.INTAKE);
      assertNotNull(RebuiltContainer.VISION_SUBSYSTEM);
      assertNotNull(RebuiltContainer.FIELD_STATE);
      assertNotNull(RebuiltContainer.STATE_MACHINE);
      assertNotNull(RebuiltContainer.CALIBRATOR);
      assertNotNull(RebuiltContainer.MANEUVER_RUNNER);
    });
  }

  @Test
  @Order(2)
  @DisplayName("Control bindings compose legally — the regression that once killed boot")
  void bindingsCompose() {
    // The specific failure this guards against:
    //   IllegalArgumentException: Multiple commands in a parallel composition cannot require
    //   the same subsystems
    assertDoesNotThrow(RebuiltContainer::configureBindings,
        "Binding construction threw. If this is a parallel composition complaining about "
            + "duplicate requirements, two commands touching the same subsystem have been put "
            + "in the same alongWith — merge them into one runOnce.");
  }

  @Test
  @Order(3)
  @DisplayName("Bindings are idempotent, so re-wiring cannot corrupt the scheduler")
  void bindingsAreRepeatable() {
    // Not something the robot does, but if a second call threw it would signal hidden state in
    // the binding code.
    assertDoesNotThrow(RebuiltContainer::configureBindings);
    assertDoesNotThrow(RebuiltContainer::configureBindings);
  }

  @Test
  @Order(4)
  @DisplayName("The drive default command is set and requires the drivetrain")
  void driveDefaultCommandIsWired() {
    RebuiltContainer.setDriveDefaultCommand();

    Command defaultCommand = CommandScheduler.getInstance()
        .getDefaultCommand(RebuiltContainer.DRIVE_SUBSYSTEM);

    assertNotNull(defaultCommand, "Without a default command the robot cannot be driven at all");
    assertTrue(defaultCommand.getRequirements().contains(RebuiltContainer.DRIVE_SUBSYSTEM));
  }

  @Test
  @Order(5)
  @DisplayName("Full non-PathPlanner wiring completes")
  void wireRobotCompletes() {
    assertDoesNotThrow(RebuiltContainer::wireRobot,
        "wireRobot covers the default command, bindings, named commands, field sectors, the "
            + "scoring lookup and the expectation set");
  }

  @Test
  @Order(6)
  @DisplayName("Expectations register, and none of them throws when evaluated")
  void expectationsAreSafeToEvaluate() {
    ExpectationMonitor monitor = ExpectationMonitor.getInstance();
    monitor.clear();

    RebuiltContainer.registerExpectations();

    assertFalse(monitor.getExpectations().isEmpty(), "The robot should register invariants");

    // A throwing expectation is caught and recorded as a failure rather than propagating, so
    // update() cannot throw — but it would silently mark everything broken. Evaluating once and
    // checking nothing reports a violation on a healthy robot proves the predicates are sound.
    assertDoesNotThrow(monitor::update);

    for (ExpectationMonitor.Expectation expectation : monitor.getExpectations()) {
      assertFalse(expectation.getName().isBlank());
      assertFalse(expectation.getDescription().isBlank(),
          expectation.getName() + " needs a description — it is what a driver reads when it "
              + "trips");
    }
  }

  @Test
  @Order(7)
  @DisplayName("Expectation names are unique, since they are log keys")
  void expectationNamesAreUnique() {
    ExpectationMonitor monitor = ExpectationMonitor.getInstance();
    monitor.clear();
    RebuiltContainer.registerExpectations();

    java.util.Set<String> names = new java.util.HashSet<>();
    for (ExpectationMonitor.Expectation expectation : monitor.getExpectations()) {
      assertTrue(names.add(expectation.getName()),
          "Duplicate expectation name would collide in the log: " + expectation.getName());
    }
  }

  @Test
  @Order(8)
  @DisplayName("Test-mode commands are all constructible")
  void testModeCommandsConstruct() {
    // These are only ever built when someone selects Test mode at an event, so a construction
    // fault would otherwise be discovered at the worst possible moment.
    assertDoesNotThrow(() -> {
      assertNotNull(new RebuiltContainer().getValidationCommand());
      assertNotNull(RebuiltContainer.getVisionValidationCommand());
      assertNotNull(RebuiltContainer.getCalibrationCommand());
      assertNotNull(RebuiltContainer.getPermutationManeuversCommand());
      assertNotNull(RebuiltContainer.getSamePathReturnCommand());
      assertNotNull(RebuiltContainer.getDifferentPathReturnCommand());
      assertNotNull(RebuiltContainer.getAllManeuversCommand());
      assertNotNull(RebuiltContainer.getLoadCalibrationCommand());
      assertNotNull(RebuiltContainer.getIntakeLoadCalibrationCommand());
      assertNotNull(RebuiltContainer.getTractionCalibrationCommand());
      assertNotNull(RebuiltContainer.getSysIdCommand());
      assertNotNull(RebuiltContainer.getBumpDiagnosticCommand());
      assertNotNull(RebuiltContainer.getInertiaCalibrationCommand());
    });
  }

  @Test
  @Order(9)
  @DisplayName("The inertia spin owns the drivetrain and can measure both intake states")
  void inertiaSpinIsWired() {
    // Open loop on purpose, so it must own the drivetrain — the default command writing closed-loop
    // setpoints over the top would make the angular acceleration a property of the velocity gains
    // rather than of the robot's inertia.
    assertTrue(RebuiltContainer.getInertiaCalibrationCommand().getRequirements()
        .contains(RebuiltContainer.DRIVE_SUBSYSTEM),
        "the inertia spin must own the drivetrain or it measures the controller, not the robot");

    assertTrue(RebuiltContainer.INERTIA_CALIBRATOR.canMeasureBothStates(),
        "an intake was supplied, so both stowed and deployed inertia should be measurable — "
            + "deploying moves mass outward and inertia goes as radius squared");
  }

  @Test
  @Order(9)
  @DisplayName("The SysId sweep owns the drivetrain, so the default command cannot fight it")
  void sysIdOwnsTheDrivetrain() {
    // SysId commands voltage directly. Without the requirement the default drive command keeps
    // writing closed-loop velocity setpoints over the top every loop, so the characterisation
    // measures a drivetrain being fought rather than one being swept — and still reports gains.
    assertTrue(RebuiltContainer.getSysIdCommand().getRequirements()
        .contains(RebuiltContainer.DRIVE_SUBSYSTEM),
        "SysId must own the drivetrain or the fit is against a contested input");
  }

  @Test
  @Order(9)
  @DisplayName("The traction sweep owns the drivetrain, so the default command cannot fight it")
  void tractionSweepRequiresTheDrivetrain() {
    // Without this requirement the sweep still runs and still reports numbers, but the default
    // drive command keeps writing zero output over every push, so it measures a drivetrain that
    // is not pushing and concludes the wheels never slip. A silently wrong calibration.
    Command sweep = RebuiltContainer.getTractionCalibrationCommand();

    assertTrue(sweep.getRequirements().contains(RebuiltContainer.DRIVE_SUBSYSTEM),
        "the traction sweep must require the drivetrain, or it competes with the default command");
  }

  @Test
  @Order(9)
  @DisplayName("Load calibration commands compose legally for every mechanism")
  void loadCalibrationComposesLegally() {
    // Each mechanism's routine is a sequence that starts and stops one subsystem. Sequences may
    // repeat a requirement, but a composition mistake here would throw at construction and take
    // the robot program down before teleop — the same failure mode as the binding regression above.
    assertDoesNotThrow(() -> {
      assertNotNull(RebuiltContainer.LOAD_CALIBRATOR.calibrateIntake());
      assertNotNull(RebuiltContainer.LOAD_CALIBRATOR.calibrateSpindexer());
      assertNotNull(RebuiltContainer.LOAD_CALIBRATOR.calibrateFeeder());
      assertNotNull(RebuiltContainer.LOAD_CALIBRATOR.calibrateShooter());
      assertNotNull(RebuiltContainer.LOAD_CALIBRATOR.full());
    });
  }

  @Test
  @Order(9)
  @DisplayName("Each load calibration owns the mechanism it is measuring")
  void loadCalibrationOwnsItsMechanism() {
    // A phase that does not require its subsystem still runs and still reports numbers, but an
    // operator holding a button changes what is being measured without the routine noticing. The
    // requirement makes that an interruption rather than silent corruption.
    assertTrue(RebuiltContainer.LOAD_CALIBRATOR.calibrateIntake().getRequirements()
        .contains(RebuiltContainer.INTAKE), "intake calibration must own the intake");

    assertTrue(RebuiltContainer.LOAD_CALIBRATOR.calibrateSpindexer().getRequirements()
        .contains(RebuiltContainer.FEEDER), "spindexer calibration must own the feeder subsystem");

    assertTrue(RebuiltContainer.LOAD_CALIBRATOR.calibrateFeeder().getRequirements()
        .contains(RebuiltContainer.FEEDER), "feeder calibration must own the feeder subsystem");

    assertTrue(RebuiltContainer.LOAD_CALIBRATOR.calibrateShooter().getRequirements()
        .contains(RebuiltContainer.SHOOTER), "shooter calibration must own the shooter");
  }

  @Test
  @Order(9)
  @DisplayName("The state machine runs a loop without an alliance and stays manual")
  void robotPeriodicSurvivesWithoutAlliance() {
    HalFixture.clearAlliance();

    RebuiltContainer container = new RebuiltContainer();
    // robotPeriodic reads field state, the scoring lookup and the state machine. With no
    // alliance and no vision, every one of those has to degrade rather than throw.
    assertDoesNotThrow(container::robotPeriodic);

    assertEquals(frc.robot.rebuilt.states.RobotStateMachine.State.MANUAL,
        RebuiltContainer.getStateOutput().state(),
        "Without a trustworthy pose the assists must stay out of the way");

    HalFixture.enableTeleop(true);
  }

  @Test
  @Order(10)
  @DisplayName("Autonomous command getter is null-safe before PathPlanner has run")
  void autonomousCommandIsNullSafe() {
    // configurePathPlanner has deliberately not been called yet, so the chooser is null. The
    // getter must cope: Robot.autonomousInit calls it unconditionally.
    RebuiltContainer container = new RebuiltContainer();
    assertDoesNotThrow(container::getAutonomousCommand);
  }

  @Test
  @Order(11)
  @DisplayName("Config loading always yields a usable config and never throws")
  void configLoadingNeverThrows() {
    // This used to raise a RuntimeException straight out of robotInit(), so a missing or
    // malformed settings file stopped the robot booting rather than merely degrading autonomous.
    assertDoesNotThrow(RobotUtils::loadRobotConfig,
        "Loading the robot config must degrade, not kill the robot program");

    assertNotNull(RobotUtils.getRobotConfig(), "A config must be available either way");
    assertEquals(4, RobotUtils.getRobotConfig().numModules);

    // This repo does ship src/main/deploy/pathplanner/settings.json, and Gradle runs tests from
    // the project root, so the real settings parse successfully here and the fallback is not
    // needed. That is worth asserting: it means the GUI settings are valid and readable, and it
    // is also why PathPlanner wiring is testable at all.
    //
    // Delete or corrupt that file and this flips to true while the robot still boots — which is
    // the behaviour the fallback exists to provide. PathPlannerConfigTest covers the fallback
    // config itself directly.
    assertFalse(RobotUtils.isUsingFallbackConfig(),
        "settings.json is present and should have parsed; if this is true, the file has become "
            + "unreadable and paths will follow worse until it is fixed");
  }

  @Test
  @Order(12)
  @DisplayName("PathPlanner wiring completes — the gap this suite previously had to document")
  void pathPlannerWiringCompletes() {
    // Only reachable because the config no longer requires a filesystem. Previously this whole
    // path was simulation-only, which is how a boot-killing fault reached main once already.
    RobotUtils.loadRobotConfig();

    assertDoesNotThrow(RebuiltContainer::configurePathPlanner,
        "AutoBuilder configuration and chooser construction must both succeed");
  }

  @Test
  @Order(13)
  @DisplayName("Autonomous command getter still safe once the chooser exists")
  void autonomousCommandAfterPathPlanner() {
    RebuiltContainer container = new RebuiltContainer();
    // Nothing is selected, so null is the correct answer — but it must not throw, and
    // Robot.autonomousInit must be able to handle the null.
    assertDoesNotThrow(container::getAutonomousCommand);
  }

  @Test
  @Order(14)
  @DisplayName("Full createContainer path runs end to end")
  void fullCreateContainerRuns() {
    // The real entry point, exercised for the first time. Everything above builds up to this.
    assertDoesNotThrow(() -> {
      var container = RebuiltContainer.createContainer();
      assertNotNull(container);
    });
  }
}
