// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import java.util.Optional;
import java.util.OptionalDouble;

import com.strubium.ssjprofiler.Profiler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CommonConstants;
import frc.robot.CommonConstants.HIDConstants;
import frc.robot.CommonConstants.ModuleConstants;
import frc.robot.common.annotations.Robot;
import frc.robot.common.components.RobotUtils;
import frc.robot.common.components.diagnostics.CalibrationManeuvers;
import frc.robot.common.components.diagnostics.CalibrationStore;
import frc.robot.common.components.diagnostics.DriftMonitor;
import frc.robot.common.components.diagnostics.DriveAutoCalibrator;
import frc.robot.common.components.diagnostics.ArmProfileCalibrator;
import frc.robot.common.components.diagnostics.CalibrationSteps;
import frc.robot.common.components.diagnostics.GuidedCalibration;
import frc.robot.common.components.diagnostics.HandMotionRoutine;
import frc.robot.common.components.diagnostics.DeployTravelCalibrator;
import frc.robot.common.components.diagnostics.DriveSysId;
import frc.robot.common.components.diagnostics.ExpectationMonitor;
import frc.robot.common.components.diagnostics.BumpCrossingDiagnostic;
import frc.robot.common.components.diagnostics.LoadCalibrationRoutine;
import frc.robot.common.components.diagnostics.ManeuverRunner;
import frc.robot.common.components.diagnostics.RotationalInertiaCalibrator;
import frc.robot.common.components.diagnostics.TractionCalibrator;
import frc.robot.rebuilt.states.RobotStateMachine;
import frc.robot.common.interfaces.IRobotContainer;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.common.subsystems.vision.VisionConstants;
import frc.robot.common.subsystems.vision.VisionSubsystem;
import frc.robot.rebuilt.RebuiltConstants.CanIds;
import frc.robot.rebuilt.RebuiltConstants.IntakeConstants;
import frc.robot.rebuilt.RebuiltConstants.ShooterConstants;
import frc.robot.rebuilt.components.FieldState;
import frc.robot.rebuilt.components.RobotSector;
import frc.robot.rebuilt.subsystems.Feeder;
import frc.robot.rebuilt.subsystems.Intake;
import frc.robot.rebuilt.subsystems.JamClearing;
import frc.robot.rebuilt.subsystems.Shooter;
import frc.robot.rebuilt.subsystems.Shooter.ShooterPosition;
import frc.robot.rebuilt.subsystems.smart.RobotSectorEvaluator;
import frc.robot.rebuilt.subsystems.smart.ScoringLocationLookup;

import lombok.AccessLevel;
import lombok.NoArgsConstructor;
import org.littletonrobotics.junction.Logger;


// Package-private rather than private so ContainerWiringTest can instantiate it and exercise the
// instance methods Robot calls every loop — robotPeriodic in particular. Still not public: only
// createContainer() and same-package tests can build one.
@NoArgsConstructor(access = AccessLevel.PACKAGE)
@Robot(team = 1745)
public class RebuiltContainer implements IRobotContainer {

  /** Owns alliance and hub state. Refreshed every loop, in every mode. */
  public static final FieldState FIELD_STATE = new FieldState();

  public static final Shooter SHOOTER =
      new Shooter(CanIds.SHOOTER_LEADER, CanIds.SHOOTER_FOLLOWER, FIELD_STATE::isHubActive);
  public static final Feeder FEEDER = new Feeder(CanIds.FEEDER, CanIds.SPINDEXER);
  public static final Intake INTAKE =
      new Intake(CanIds.INTAKE_ROLLER_1, CanIds.INTAKE_ROLLER_2, CanIds.INTAKE_DEPLOY);
  public static final SwerveDriveSubsystem DRIVE_SUBSYSTEM = new SwerveDriveSubsystem();

  /**
   * AprilTag localisation and calibration.
   *
   * <p>Safe to construct with no camera plugged in — it simply contributes nothing. Before
   * trusting it, check the three values marked MEASURE in
   * {@link frc.robot.common.subsystems.vision.VisionConstants}.
   */
  public static final VisionSubsystem VISION_SUBSYSTEM = new VisionSubsystem(
      VisionConstants.CAMERA_NAME,
      DRIVE_SUBSYSTEM::addVisionMeasurement,
      DRIVE_SUBSYSTEM::getPose,
      DRIVE_SUBSYSTEM::getOdometryOnlyPose,
      DRIVE_SUBSYSTEM::getHeading,
      DRIVE_SUBSYSTEM::getChassisSpeedMetersPerSecond);

  public static final RobotSectorEvaluator SECTOR_EVALUATOR = new RobotSectorEvaluator(DRIVE_SUBSYSTEM);

  private static SendableChooser<Command> automodeChooser;

  /** Localisation-driven behaviour selection. Updated every loop in robotPeriodic. */
  public static final RobotStateMachine STATE_MACHINE = new RobotStateMachine();

  /** The most recent state decision. Read by the drive default command for heading assist. */
  private static RobotStateMachine.StateOutput stateOutput = new RobotStateMachine.StateOutput(
      RobotStateMachine.State.MANUAL, Optional.empty(), OptionalDouble.empty(), "not yet run");

  /** Drivetrain calibration, run from Test mode. */
  public static final DriveAutoCalibrator CALIBRATOR =
      new DriveAutoCalibrator(DRIVE_SUBSYSTEM, VISION_SUBSYSTEM);

  /** Manoeuvre suite runner, run from Test mode. */
  public static final ManeuverRunner MANEUVER_RUNNER = new ManeuverRunner(DRIVE_SUBSYSTEM);

  /** Current-threshold calibration for piece and jam detection, run from Test mode. */
  public static final LoadCalibrationRoutine LOAD_CALIBRATOR =
      new LoadCalibrationRoutine(INTAKE, FEEDER, SHOOTER);

  /** Drive current limit calibration, run against a wall on carpet. */
  public static final TractionCalibrator TRACTION_CALIBRATOR =
      new TractionCalibrator(DRIVE_SUBSYSTEM);

  /** WPILib SysId, chained and self-analysing. The only source of kA. */
  public static final DriveSysId SYSID = new DriveSysId(DRIVE_SUBSYSTEM);

  /**
   * Measures rotational inertia, which CAD cannot supply on this robot.
   *
   * <p>Measures both intake states, because deploying moves several kilograms outward and inertia
   * goes as mass times radius squared.
   */
  public static final RotationalInertiaCalibrator INERTIA_CALIBRATOR =
      new RotationalInertiaCalibrator(DRIVE_SUBSYSTEM, INTAKE::stopDeploy, INTAKE::manualDeploy);

  /** Measures what the intake arm's motion profile depends on. */
  public static final ArmProfileCalibrator ARM_PROFILE = new ArmProfileCalibrator(INTAKE);

  /**
   * The hand-motion check: every mechanism moved by hand, motors unpowered.
   *
   * <p>Advanced by the operator controller's <b>A</b> button. Any button would do — the latch only
   * needs a boolean — and A is chosen because it is the one an operator can find without looking while
   * both hands are on the robot.
   */
  public static final HandMotionRoutine HAND_MOTION = new HandMotionRoutine(
      DRIVE_SUBSYSTEM, INTAKE, FEEDER, SHOOTER,
      () -> HIDConstants.OPERATOR_CONTROLLER.a().getAsBoolean());

  /** Measures the intake arm's real travel against its physical stops. */
  public static final DeployTravelCalibrator DEPLOY_TRAVEL = new DeployTravelCalibrator(INTAKE);

  /** Diagnoses why the chassis bogs down on the field ramps. */
  public static final BumpCrossingDiagnostic BUMP_DIAGNOSTIC =
      new BumpCrossingDiagnostic(DRIVE_SUBSYSTEM, VISION_SUBSYSTEM::hasRecentMeasurement);

  /**
   * Watches live estimates against the constants in use, so break-in is noticed rather than
   * silently absorbed into gain tuning.
   *
   * <p>Observes only. Promotion is {@link #getPromoteCalibrationCommand(String)}, run
   * deliberately.
   */
  public static final DriftMonitor DRIFT_MONITOR = new DriftMonitor();

  public static IRobotContainer createContainer() {
    // Split in two so the half that does not need PathPlanner can be exercised by a test.
    // The composition error that once killed the robot program at boot lived in
    // configureBindings(), and it was invisible to every test because none of this was
    // reachable without a deploy directory.
    wireRobot();
    configurePathPlanner();

    return new RebuiltContainer();
  }

  /**
   * All wiring that does not depend on PathPlanner: the drive default command, control
   * bindings, field geometry and the expectation set.
   *
   * <p>Package-private so {@code ContainerWiringTest} can call it in a JVM with no deploy
   * directory. Everything here throws at construction time if it is malformed, which is
   * precisely the class of fault worth catching before a robot is switched on.
   */
  static void wireRobot() {
    setDriveDefaultCommand();

    // Bind buttons and triggers
    configureBindings();

    // Register named commands
    registerNamedCommands();

    // TODO: these four sectors are placeholder test data — all BLUE/TOWER in a 1x1 grid
    // near the origin. Replace with real field geometry before competition.
    SECTOR_EVALUATOR.createSector(RobotSector.BaseSector.BLUE, RobotSector.SectorType.TOWER, new Pose2d(1., 1., new Rotation2d()), 1, 1);
    SECTOR_EVALUATOR.createSector(RobotSector.BaseSector.BLUE, RobotSector.SectorType.TOWER, new Pose2d(3., 1., new Rotation2d()), 1, 1);
    SECTOR_EVALUATOR.createSector(RobotSector.BaseSector.BLUE, RobotSector.SectorType.TOWER, new Pose2d(1., 3., new Rotation2d()), 1, 1);
    SECTOR_EVALUATOR.createSector(RobotSector.BaseSector.BLUE, RobotSector.SectorType.TOWER, new Pose2d(3., 3., new Rotation2d()), 1, 1);

    // Set up scoring location lookup
    ScoringLocationLookup.buildScoringLocations();

    registerExpectations();
    registerDriftWatches();
  }

  /**
   * Registers the quantities worth watching for break-in.
   *
   * <p>Idempotent, like the other wiring steps. Thresholds are chosen so that normal noise stays
   * quiet: a monitor that reports every half-percent wobble trains everyone to ignore it, and then
   * the real drift goes unnoticed too.
   */
  static void registerDriftWatches() {
    DRIFT_MONITOR.clear();

    var calibration = VISION_SUBSYSTEM.getCalibration();

    // Wheel diameter. The live estimate is the AprilTag-derived wheel scale applied to the
    // constant currently in use. 1% over 10 ft is 30 mm — more than the whole accuracy budget —
    // so 1% is the right threshold, and 200 samples keeps it honest.
    DRIFT_MONITOR.watch(
        "drive.wheelDiameter", "m",
        () -> ModuleConstants.kWheelDiameterMeters,
        () -> ModuleConstants.kWheelDiameterMeters * calibration.getWheelScaleEstimate(),
        calibration::getSampleCount,
        0.01,
        200);

    // Vision translational noise. Wider threshold because the measurement itself is noisy, and it
    // is the one value here that may be adopted without a human — see CalibrationStore.mayAutoAdopt.
    DRIFT_MONITOR.watch(
        "vision.noise.xyStdDev", "m",
        () -> VisionConstants.SINGLE_TAG_XY_STD_DEV_BASE,
        calibration::getMeasuredXyStdDevMeters,
        calibration::getSampleCount,
        0.25,
        300);

    // Gyro scale. A 1% scale error is 3.6 degrees per revolution, which compounds through every
    // turn in an autonomous path.
    DRIFT_MONITOR.watch(
        "gyro.scale", "ratio",
        () -> 1.0,
        () -> 1.0 + calibration.getGyroYawError().getMean() / 360.0,
        () -> calibration.getGyroYawError().getCount(),
        0.01,
        200);
  }

  /**
   * Sets the drivetrain's default command.
   *
   * <p>The suppliers MUST be method references, not evaluated values: passing
   * {@code controller.getLeftX()} directly captured a single reading taken at class-load time,
   * so the drivetrain was commanded to a frozen zero for the whole match.
   *
   * <p>LeftY drives the x request and LeftX the y request because the field frame has +x forward
   * and +y left, while the stick reports +y backward and +x right — hence the negations.
   *
   * <p>Heading assist comes from the localisation state machine: aim at the hub on our own side
   * during our shift, and turn to cross the bump backwards. Translation always stays with the
   * driver, and touching the rotation stick overrides the assist instantly.
   */
  static void setDriveDefaultCommand() {
    DRIVE_SUBSYSTEM.setDefaultCommand(
      DRIVE_SUBSYSTEM.driveCommandWithHeadingAssist(
        () -> -HIDConstants.DRIVER_CONTROLLER.getLeftY(),
        () -> -HIDConstants.DRIVER_CONTROLLER.getLeftX(),
        () -> -HIDConstants.DRIVER_CONTROLLER.getRightX(),
        () -> stateOutput.headingTarget(),
        true)
    );
  }

  /**
   * PathPlanner setup, which needs settings from the deploy directory.
   *
   * <p>Separated out because {@code RobotConfig.fromGUISettings()} has nothing to read in a unit
   * test JVM, and that single dependency is what previously made the whole of
   * {@link #createContainer()} untestable.
   */
  static void configurePathPlanner() {
    DRIVE_SUBSYSTEM.configureAutoBuilder();

    automodeChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(
        CommonConstants.SmartDashboardConstants.SMARTDASHBOARD_AUTO_MODE, automodeChooser);
  }

  /**
   * Registers the invariants that say "this robot is working".
   *
   * <p>These are observed every loop and logged under {@code Expectations/}. They are the
   * automated form of the checks a driver would otherwise make by feel, and they exist
   * because the most serious bug in this codebase — a drivetrain that ignored the sticks —
   * was invisible without them.
   */
  /**
   * Registers the invariants that say "this robot is working".
   *
   * <p>Package-private so a test can confirm the set registers cleanly and that each predicate
   * is safe to evaluate — a throwing expectation would otherwise only be discovered on the
   * field, inside {@code robotPeriodic()}.
   */
  static void registerExpectations() {
    ExpectationMonitor monitor = ExpectationMonitor.getInstance();

    // Idempotent, matching configureBindings(). The monitor is a singleton, so calling this
    // twice would otherwise register every invariant again — and since expectation names are
    // AdvantageKit log keys, the duplicates would collide silently. Only one container is ever
    // built on a robot, so clearing first is safe and makes re-wiring harmless.
    monitor.clear();

    monitor.register(
        "DriveRespondsToStick",
        "When the driver commands motion, at least one module is commanded to move",
        () -> {
          double commanded = Math.abs(SwerveDriveSubsystem.applyDeadband(
                  HIDConstants.DRIVER_CONTROLLER.getLeftY()))
              + Math.abs(SwerveDriveSubsystem.applyDeadband(
                  HIDConstants.DRIVER_CONTROLLER.getLeftX()))
              + Math.abs(SwerveDriveSubsystem.applyDeadband(
                  HIDConstants.DRIVER_CONTROLLER.getRightX()));
          if (commanded <= 0) {
            return true; // Nothing asked for, nothing to check.
          }
          for (var state : DRIVE_SUBSYSTEM.getDesiredModuleStates()) {
            if (Math.abs(state.speedMetersPerSecond) > 1e-3) {
              return true;
            }
          }
          return false;
        },
        5);

    monitor.register(
        "HeadingFinite",
        "The gyro is reporting a usable heading",
        () -> Double.isFinite(DRIVE_SUBSYSTEM.getHeading()));

    monitor.register(
        "PoseFinite",
        "The pose estimate has not diverged to NaN or infinity",
        () -> {
          Pose2d pose = DRIVE_SUBSYSTEM.getPose();
          return Double.isFinite(pose.getX())
              && Double.isFinite(pose.getY())
              && Double.isFinite(pose.getRotation().getRadians());
        });

    monitor.register(
        "ShooterReachesSetpoint",
        "Once commanded, the flywheel arrives at its target RPM and stays there",
        () -> !SHOOTER.isRunning() || SHOOTER.isAtTargetRPM(),
        // Generous: spin-up from rest to 4500 RPM legitimately takes a few seconds.
        150);

    monitor.register(
        "ShooterRespectsHubInterlock",
        "The flywheel is never commanded while our hub is closed",
        () -> SHOOTER.isHubActive() || !SHOOTER.isRunning());

    monitor.register(
        "IntakeWithinSoftLimits",
        "The deploy arm stays inside its configured travel",
        () -> {
          double position = INTAKE.getDeployPosition();
          return position >= IntakeConstants.DEPLOY_REVERSE_SOFT_LIMIT - 1.0
              && position <= IntakeConstants.DEPLOY_FORWARD_SOFT_LIMIT + 1.0;
        });

    monitor.register(
        "VisionNotDivergingFromOdometry",
        "Vision and wheel odometry agree to within a metre",
        () -> {
          // A large, sustained gap means one of them is wrong: a bad camera transform, the
          // wrong field layout (welded vs AndyMark), or genuinely bad wheel calibration.
          // Only meaningful once vision is actually contributing.
          if (!VISION_SUBSYSTEM.hasRecentMeasurement()) {
            return true;
          }
          return DRIVE_SUBSYSTEM.getPose().getTranslation()
              .getDistance(DRIVE_SUBSYSTEM.getOdometryOnlyPose().getTranslation()) < 1.0;
        },
        // Generous: a legitimate correction after a bad starting pose is a big, brief jump.
        250);

    monitor.register(
        "BallPathNotJammed",
        "No mechanism in the ball path is loaded but stalled",
        () -> !INTAKE.isJammed() && !FEEDER.isJammed(),
        // Generous: a jam only counts once the load monitors have confirmed it, and the clearing
        // routine gets a chance to fix it before this is worth flagging to a driver.
        100);

    monitor.register(
        "LoadBaselinesLearned",
        "Current baselines are established, so piece and jam detection means something",
        () -> {
          // Only meaningful once the mechanisms have actually been run; before that, an
          // unestablished baseline is expected rather than a fault.
          if (!INTAKE.isRunning() && !FEEDER.isLoading()) {
            return true;
          }
          return INTAKE.getRollerLoad().isBaselineEstablished()
              || FEEDER.getFeederLoad().isBaselineEstablished();
        },
        250);

    monitor.register(
        "VisionRejectRateReasonable",
        "Most vision measurements pass the plausibility gates",
        () -> {
          int accepted = VISION_SUBSYSTEM.getAcceptedCount();
          int rejected = VISION_SUBSYSTEM.getRejectedCount();
          if (accepted + rejected < 50) {
            return true; // Not enough data to judge.
          }
          // Rejecting nearly everything usually means a wrong camera transform putting every
          // solved pose off the field.
          return accepted > rejected / 4;
        },
        50);
  }


  /**
   * Binds every driver and operator control.
   *
   * <p>Package-private rather than private specifically so a test can call it. WPILib rejects an
   * illegal command composition at <em>construction</em> time, and because this runs inside a
   * static initialiser that exception kills the robot program before teleop is ever reached —
   * which is exactly what happened when the intake commands gained subsystem requirements and
   * two of them ended up in one parallel composition. Every unit test passed; only simulation
   * caught it.
   *
   * <p>Deliberately free of PathPlanner, so it can be exercised in a test JVM that has no deploy
   * directory. {@link #configurePathPlanner()} holds that part.
   */
  static void configureBindings() {
    Profiler bindingProfiler = new Profiler("bindings");
    bindingProfiler.start();

    // Operator POV Down - Lower Shooter RPM trim
    RobotUtils.bindControl(HIDConstants.OPERATOR_CONTROLLER.povDown(),
      Commands.runOnce(() -> SHOOTER.lowerOperatorModifer(ShooterConstants.OPERATOR_TRIM_STEP_RPM)),
      Commands.none());

    // Operator POV Up - Raise Shooter RPM trim
    RobotUtils.bindControl(HIDConstants.OPERATOR_CONTROLLER.povUp(),
      Commands.runOnce(() -> SHOOTER.raiseOperatorModifer(ShooterConstants.OPERATOR_TRIM_STEP_RPM)),
      Commands.none());

    // Driver Right Stick Button - Reset heading
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.rightStick(), Commands.runOnce(DRIVE_SUBSYSTEM::zeroHeading, DRIVE_SUBSYSTEM), Commands.none());

    //Driver DPad Up - Deploy intake
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.povUp(),
      INTAKE.deploy(),
      Commands.runOnce(INTAKE::stopDeploy, INTAKE));

    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.povDown(),
    Commands.runOnce(INTAKE::manualReverseDeploy, INTAKE),
    Commands.runOnce(INTAKE::stopDeploy, INTAKE));

    // Driver A Button - Shoot from hub
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.a(),
            shootFrom(ShooterPosition.HUB),
            Commands.runOnce(SHOOTER::idleOrStop));

    // Driver B Button - Shoot from trench
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.b(),
            shootFrom(ShooterPosition.TRENCH),
            Commands.runOnce(SHOOTER::idleOrStop));

    // Driver X Button - Shoot from tower
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.x(),
            shootFrom(ShooterPosition.TOWER),
            Commands.runOnce(SHOOTER::idleOrStop));

    // Driver Y Button - Shoot from corner
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.y(),
            shootFrom(ShooterPosition.CORNER),
            Commands.runOnce(SHOOTER::idleOrStop));


    // Driver Right Bumper - Reverse Load
    RobotUtils.bindControl(
            HIDConstants.DRIVER_CONTROLLER.rightBumper(),
            Commands.runOnce(() -> {
              FEEDER.reverseLoad();
              FEEDER.reverseCycle();
            }, FEEDER),
            Commands.runOnce(() -> {
              FEEDER.stopLoad();
              FEEDER.holdCycle();
            }, FEEDER));

    // Driver Right Trigger - Load (fire balls if shooter is on)
    //
    // Both intake actions must live in a SINGLE command: a parallel composition cannot
    // contain two commands that require the same subsystem, and now that these commands
    // declare INTAKE as a requirement, splitting them throws at construction time.
    RobotUtils.bindControl(
            HIDConstants.DRIVER_CONTROLLER.rightTrigger(),
            Commands.runOnce(() -> {
              FEEDER.load();
              FEEDER.cycle();
            }, FEEDER).alongWith(
              Commands.runOnce(() -> {
                INTAKE.manualReverseDeploy();
                INTAKE.intake();
              }, INTAKE)),
            Commands.runOnce(() -> {
              FEEDER.stopLoad();
              FEEDER.holdCycle();
            }, FEEDER).alongWith(
              Commands.runOnce(() -> {
                INTAKE.stopDeploy();
                INTAKE.holdRollers();
              }, INTAKE)));

    // Driver Left Trigger - Intake
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.leftTrigger(),
      Commands.runOnce(INTAKE::intake, INTAKE),
      Commands.runOnce(INTAKE::holdRollers, INTAKE));

    // Driver Left Bumper - Outtake
    RobotUtils.bindControl(HIDConstants.DRIVER_CONTROLLER.leftBumper(),
      Commands.runOnce(INTAKE::outtake, INTAKE),
      Commands.runOnce(INTAKE::stopRollers, INTAKE));

    // Operator Back - clear the whole ball path on demand.
    //
    // Bound to a button as well as being available automatically, because a driver often knows a
    // piece is stuck before the current signature confirms it — and because a manual override
    // needs to exist for when the detection thresholds turn out to be wrong.
    HIDConstants.OPERATOR_CONTROLLER.back().onTrue(
        JamClearing.clearWholePath(INTAKE, FEEDER,
            () -> INTAKE.isJammed() || FEEDER.isJammed()));

    // Operator Start - jostle just the intake, the most common case.
    HIDConstants.OPERATOR_CONTROLLER.start().onTrue(
        JamClearing.intakeJostle(INTAKE, INTAKE::isJammed));

    bindingProfiler.end();
  }

  /**
   * Builds the command that selects a shooter preset and holds the flywheel there.
   *
   * @param position Preset to select.
   * @return A command suitable for {@code whileTrue}.
   */
  private static Command shootFrom(ShooterPosition position) {
    return Commands.runOnce(() -> SHOOTER.setCurrentShooterPosition(position))
        .alongWith(Commands.run(SHOOTER::runShooter, SHOOTER));
  }

  private static void registerNamedCommands() {
    NamedCommands.registerCommand("Set Shooter Hub", Commands.runOnce(() -> SHOOTER.setCurrentShooterPosition(ShooterPosition.HUB)));
    NamedCommands.registerCommand("Set Shooter Trench", Commands.runOnce(() -> SHOOTER.setCurrentShooterPosition(ShooterPosition.TRENCH)));
    NamedCommands.registerCommand("Set Shooter Corner", Commands.runOnce(() -> SHOOTER.setCurrentShooterPosition(ShooterPosition.CORNER)));
    NamedCommands.registerCommand("Set Shooter Tower", Commands.runOnce(() -> SHOOTER.setCurrentShooterPosition(ShooterPosition.TOWER)));
  }

  @Override
  public void simulationPeriodic() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void robotInit() {
  }

  /**
   * Refreshes field state and driver-assist telemetry in every mode.
   *
   * <p>All of this used to live in {@link #teleopPeriodic()}, which meant the hub interlock
   * was stale throughout autonomous and every read of the alliance risked throwing.
   */
  @Override
  public void robotPeriodic() {
    FIELD_STATE.update();

    ScoringLocationLookup.setRedAlliance(FIELD_STATE.isAllianceRed());

    Logger.recordOutput("Assist/ShooterPosition",
        ScoringLocationLookup.findClosest(DRIVE_SUBSYSTEM.getPose()));
    Logger.recordOutput("Assist/HubPose", ScoringLocationLookup.findHub());
    Logger.recordOutput("Field/GameDataNamesUs", FIELD_STATE.gameDataNamesUs());

    updateStateMachine();
  }

  /**
   * Selects the localisation-driven behaviour for this loop.
   *
   * <p>The pose is only trusted when an alliance is known and vision has contributed recently.
   * Without both, the state machine falls back to manual: acting confidently on a bad pose is
   * worse than not acting, and a heading assist that fights the driver based on a wrong pose is
   * the most frustrating possible failure.
   */
  private static void updateStateMachine() {
    ChassisSpeeds speeds = DRIVE_SUBSYSTEM.getChassisSpeeds();
    // Robot-relative speeds rotated into the field frame give the actual travel direction,
    // which is what the bump logic needs — a swerve chassis can drive one way while facing
    // another.
    Translation2d travelDirection = new Translation2d(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        .rotateBy(DRIVE_SUBSYSTEM.getPose().getRotation());

    boolean poseTrustworthy = FIELD_STATE.hasAlliance() && VISION_SUBSYSTEM.hasRecentMeasurement();

    stateOutput = STATE_MACHINE.update(
        DRIVE_SUBSYSTEM.getPose(),
        travelDirection,
        ScoringLocationLookup.findHub(),
        VISION_SUBSYSTEM.getFieldLayout().getFieldLength(),
        FIELD_STATE.isAllianceRed(),
        FIELD_STATE.isHubActive(),
        poseTrustworthy);

    // Range-based flywheel speed, applied only while the aim state is active so a driver
    // holding a preset button is never fought.
    if (stateOutput.state() == RobotStateMachine.State.AIM_AT_HUB
        && stateOutput.hasShooterTarget()) {
      SHOOTER.setRangeTargetRpm(stateOutput.shooterRpm().getAsDouble());
    } else {
      SHOOTER.clearRangeTarget();
    }

    Logger.recordOutput("States/HeadingAssistActive", DRIVE_SUBSYSTEM.isHeadingAssistActive());
    Logger.recordOutput("States/HeadingAssistErrorDeg",
        DRIVE_SUBSYSTEM.getHeadingAssistErrorDegrees());

    // Observes only; never changes a constant. Runs in every mode so evidence accumulates during
    // ordinary driving rather than only during a dedicated calibration session.
    DRIFT_MONITOR.update();
    CalibrationStore.getInstance().log();
  }

  /** @return the state selected on the most recent loop. */
  public static RobotStateMachine.StateOutput getStateOutput() {
    return stateOutput;
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopPeriodic() {
  }

  /**
   * Get currently selected autonomous command
   *
   * @return Autonomous command
   */
  @Override
  public Command getAutonomousCommand() {
    return automodeChooser == null ? null : automodeChooser.getSelected();
  }

  /**
   * The on-blocks self-test. Enter Test mode on the driver station to run it.
   *
   * @return the validation command for this robot.
   */
  @Override
  public Command getValidationCommand() {
    return RebuiltValidation.build(DRIVE_SUBSYSTEM, SHOOTER, INTAKE, FEEDER);
  }

  /**
   * The AprilTag self-test, kept separate from the mechanical one.
   *
   * <p>Needs the robot on the practice field with at least one tag in view, so it is not part
   * of the pit-side Test-mode suite. Schedule it manually, or bind it to a button while
   * commissioning vision.
   *
   * @return the vision validation command.
   */
  public static Command getVisionValidationCommand() {
    return RebuiltValidation.buildVisionChecks(VISION_SUBSYSTEM, DRIVE_SUBSYSTEM);
  }

  /**
   * The full drivetrain auto-calibration.
   *
   * <p>Needs the robot on the floor with about 4 m clear ahead and AprilTags in view. Measures
   * wheel scale, steering misalignment, gyro scale, effective drive radius and the feedforward,
   * then runs the 10 ft acceptance test both open loop and closed loop.
   *
   * @return the calibration command.
   */
  public static Command getCalibrationCommand() {
    return CALIBRATOR.full();
  }

  /**
   * The full manoeuvre suite: sixteen drive-turn-drive permutations, then the out-and-back
   * families.
   *
   * <p>Needs a lot of space and several minutes. Start with
   * {@link #getPermutationManeuversCommand()} or a single family if space is tight.
   *
   * @return the manoeuvre suite command.
   */
  public static Command getAllManeuversCommand() {
    return MANEUVER_RUNNER.runAll(CalibrationManeuvers.all());
  }

  /**
   * The current-threshold calibration for game piece and jam detection.
   *
   * <p>Needs the robot on blocks, a stack of game pieces, and a person at the mechanism to feed them
   * and then to obstruct it. Takes about a minute per mechanism. Prints recommended values for
   * {@code LoadConstants} — it writes nothing itself.
   *
   * @return the load calibration command.
   */
  public static Command getLoadCalibrationCommand() {
    return LOAD_CALIBRATOR.full();
  }

  /** @return just the intake roller thresholds, for when only that mechanism has changed. */
  public static Command getIntakeLoadCalibrationCommand() {
    return LOAD_CALIBRATOR.calibrateIntake();
  }

  /**
   * The drive current limit calibration.
   *
   * <p><b>Robot square against a wall, on carpet, on a good battery.</b> Pushes at full output while
   * stepping the drive current limit from 20 A upward until the wheels break traction, then
   * recommends a limit below that point. Aborts if the robot is not actually against the wall.
   *
   * <p>Takes about a minute. Stalls the drive motors in short bursts with cooldowns between, so it is
   * hard on the drivetrain but not damaging. Prints a recommendation for
   * {@code SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT}; the limits it applies while running do not
   * persist across a power cycle.
   *
   * @return the traction calibration command.
   */
  public static Command getTractionCalibrationCommand() {
    return TRACTION_CALIBRATOR.sweep();
  }

  /**
   * WPILib SysId across all four tests, fitted on the robot.
   *
   * <p><b>Needs about 10 m of clear runway</b> — more than anything else in the suite. Runs
   * quasistatic forward and reverse, then dynamic forward and reverse, with rests between, then
   * prints kS, kV and kA per module plus the mean. No log transfer and no desktop analyser: the
   * regression the analyser performs is done here.
   *
   * <p>This is the only source of <b>kA</b>, which second-order kinematics needs. The
   * auto-calibrator's sweep waits for steady state, so acceleration is zero in its data by
   * construction and kA is not merely unmeasured there but unmeasurable.
   *
   * <p>The log is still a standard SysId log, so the desktop analyser remains available if the
   * on-robot fit ever looks wrong and the residual plots are wanted.
   *
   * @return the SysId characterisation command.
   */
  public static Command getSysIdCommand() {
    return SYSID.full();
  }

  /**
   * Diagnoses why the chassis slows or fails on the field ramps.
   *
   * <p>Schedule it, then drive over the ramp normally — it watches rather than taking control, so
   * what gets measured is the crossing as actually driven. Classifies the run as current-limited,
   * traction-limited or voltage-limited, which need three different fixes, and says so in words.
   *
   * <p><b>Run it where an AprilTag is visible.</b> Slip is wheel speed against chassis speed, and
   * the only chassis speed on this robot that is independent of the wheels comes from the tags. With
   * no tag in view it reports the run as inconclusive rather than clearing traction wrongly.
   *
   * @return the bump diagnostic command.
   */
  public static Command getBumpDiagnosticCommand() {
    return BUMP_DIAGNOSTIC.watch();
  }

  /**
   * Measures the robot's rotational inertia, in both intake positions.
   *
   * <p><b>Robot on the floor with about 2 m clear all round.</b> Spins in place twice at a low
   * voltage, once with the intake stowed and once deployed, and reports the moment of inertia for
   * each plus the difference.
   *
   * <p>This is the number {@code settings.json} carries as {@code robotMOI} and the one that normally
   * comes from CAD — which is not available here, because the assembly is too large for the
   * mass-properties tool. Measuring it on the robot has the advantage of including the wire and tape
   * a model never has.
   *
   * <p>Rejects a run where the wheels slipped, since the torque figure assumes every newton reaches
   * the carpet.
   *
   * @return the inertia calibration command.
   */
  public static Command getInertiaCalibrationCommand() {
    return INERTIA_CALIBRATOR.full();
  }

  /**
   * Measures the intake arm's travel by driving it gently onto each hard stop.
   *
   * <p><b>On blocks, and with no game pieces in the robot</b> — a ball under the arm gets found
   * instead of the stop. It is reported as an obstruction rather than mistaken for a stop, so the run
   * fails honestly, but it is still wasted.
   *
   * <p>Closes a gap open since the first review: the deploy target of 10 rotations and the soft limits
   * of 0 to 11 are hand-chosen numbers that nothing has ever checked against the arm.
   *
   * @return the travel calibration command.
   */
  public static Command getDeployTravelCommand() {
    return DEPLOY_TRAVEL.full();
  }

  /**
   * The guided calibration: two buttons, and the robot judges its own data.
   *
   * <p><b>READY</b> (operator A) starts a measurement once the setup prompt has been satisfied.
   * <b>NEXT</b> (operator B) moves on. After each measurement the step assesses what it gathered and
   * either passes it or says what to change and offers the step again.
   *
   * <p>That assessment is the part worth having. Every routine here already knew whether its result
   * was usable -- a regression knows its R-squared and sample count, a traction sweep knows whether the
   * drivetrain ever bound -- but the knowledge was buried in printed prose, so acting on it meant a
   * human reading a console mid-session and deciding. Now a bad run says <em>re-gather, and here is
   * what to change</em>, and a good one says so plainly.
   *
   * <p>Ordered cheapest-and-most-depended-upon first: the arm travel is on blocks and everything about
   * the arm needs it, traction needs only a wall, and SysId needs the most floor.
   *
   * @return the guided calibration command.
   */
  public static Command getGuidedCalibrationCommand() {
    return new GuidedCalibration(
            () -> HIDConstants.OPERATOR_CONTROLLER.a().getAsBoolean(),
            () -> HIDConstants.OPERATOR_CONTROLLER.b().getAsBoolean())
        .add(CalibrationSteps.armTravel(DEPLOY_TRAVEL, INTAKE))
        .add(CalibrationSteps.tractionLimit(TRACTION_CALIBRATOR))
        .add(CalibrationSteps.driveFeedforward(SYSID))
        .full();
  }

  /**
   * Establishes which way every mechanism counts, and the arm's travel, <b>with the motors off</b>.
   *
   * <p><b>Run this before anything powered.</b> Every test that follows assumes it knows which way is
   * forward. A sign error makes the drive characterisation fit a negative gain and makes the arm's
   * profile drive away from its goal until it reaches steel — and both of those present as mechanical
   * faults, so they cost an afternoon on the mechanism before anyone suspects a sign. Turning each
   * thing by hand finds the same error in seconds, at no risk.
   *
   * <p><b>On blocks, and somebody has to hold the arm.</b> It is not balanced, and this routine
   * releases it deliberately.
   *
   * <p>Nothing is ever commanded to move. The robot needs to be enabled only because commands do not
   * run otherwise.
   *
   * <p>It also produces the arm's travel from its two hard stops, which makes
   * {@link #getDeployTravelCommand()} a check rather than a discovery — far better than learning the
   * span by driving the arm at a stop and watching the current.
   *
   * @return the hand-motion calibration command.
   */
  public static Command getHandMotionCommand() {
    return HAND_MOTION.full();
  }

  /**
   * Measures the intake arm's break-away voltage, velocity feedforward, gravity signature and — most
   * importantly — whether its profile constraints are achievable.
   *
   * <p><b>On blocks, no game pieces.</b> Swings the full travel several times.
   *
   * <p>Run {@link #getDeployTravelCommand()} first: the gravity phase drives to fractions of the
   * measured travel, so it is skipped if the travel is unknown.
   *
   * @return the arm profile calibration command.
   */
  public static Command getArmProfileCommand() {
    return ARM_PROFILE.full();
  }

  /**
   * Every superstructure calibration, in dependency order.
   *
   * <p><b>On blocks. Game pieces to hand, but NOT loaded when it starts</b> — the travel measurement
   * needs an empty robot and it runs first.
   *
   * <p>The order is not arbitrary. Arm travel comes first because the arm profile calibration drives
   * to fractions of it and the profile goal clamping uses the learned stops. Load thresholds come last
   * because that is the phase that needs a person feeding game pieces, so everything automatic is out
   * of the way by then.
   *
   * @return the combined superstructure calibration.
   */
  public static Command getSuperstructureCalibrationCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> System.out.println(
            "[calib] === Superstructure calibration: travel, arm profile, then load ===")),
        getDeployTravelCommand(),
        getArmProfileCommand(),
        getLoadCalibrationCommand());
  }

  /** @return just the sixteen drive-turn-drive permutations. */
  public static Command getPermutationManeuversCommand() {
    return MANEUVER_RUNNER.runAll(CalibrationManeuvers.permutations());
  }

  /** @return just the out-and-back manoeuvres that retrace their outbound path. */
  public static Command getSamePathReturnCommand() {
    return MANEUVER_RUNNER.runAll(CalibrationManeuvers.outAndBackSamePath());
  }

  /** @return just the out-and-back manoeuvres that return by a different route. */
  public static Command getDifferentPathReturnCommand() {
    return MANEUVER_RUNNER.runAll(CalibrationManeuvers.outAndBackDifferentPath());
  }

  /**
   * Prints what has drifted, without changing anything.
   *
   * <p>Safe to run any time, including between matches. Start here rather than with the promotion
   * command: read what the robot thinks has changed before accepting it.
   *
   * @return a reporting command.
   */
  public static Command getDriftReportCommand() {
    return Commands.runOnce(() -> {
      CalibrationStore.getInstance().printReport();
      DRIFT_MONITOR.printReport();
    }).ignoringDisable(true).withName("DriftReport");
  }

  /**
   * Writes the current live estimates into the persistent store.
   *
   * <p><b>Deliberate act, human present.</b> Only values with enough evidence and genuine drift are
   * promoted, and each is recorded with the date and sample count behind it so it can be judged
   * later. Values that cannot be measured independently of themselves — wheel diameter, gyro scale,
   * any gain — are still written here, but only because a person asked; nothing adopts them on its
   * own. See {@code CalibrationStore.mayAutoAdopt} for why that distinction exists.
   *
   * <p>Run {@link #getDriftReportCommand()} first, and take the robot's word for nothing that the
   * report does not back with samples.
   *
   * @param today Date stamp for the audit trail, e.g. "2026-08-04".
   * @return a promotion command.
   */
  public static Command getPromoteCalibrationCommand(String today) {
    return Commands.runOnce(() -> {
      CalibrationStore store = CalibrationStore.getInstance();
      var drifted = DRIFT_MONITOR.getDrifted();

      if (drifted.isEmpty()) {
        System.out.println("[calibration] nothing has drifted past its threshold with enough "
            + "evidence — nothing to promote");
        return;
      }

      for (var watch : drifted) {
        store.promote(
            watch.getName(),
            watch.getLiveEstimate(),
            watch.getSamples(),
            today,
            watch.mayAutoAdopt() ? "auto-adoptable" : "accepted by operator");
      }

      if (store.save()) {
        System.out.println("[calibration] promoted " + drifted.size()
            + " value(s). They take effect on the next reboot; verify with a 10 ft acceptance run "
            + "before trusting them in a match.");
      }
      store.printReport();
    }).ignoringDisable(true).withName("PromoteCalibration");
  }
}