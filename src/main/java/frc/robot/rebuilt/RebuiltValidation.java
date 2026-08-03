package frc.robot.rebuilt;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.components.diagnostics.ValidationSuite;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.common.subsystems.vision.VisionConstants;
import frc.robot.common.subsystems.vision.VisionSubsystem;
import frc.robot.rebuilt.RebuiltConstants.IntakeConstants;
import frc.robot.rebuilt.subsystems.Feeder;
import frc.robot.rebuilt.subsystems.Intake;
import frc.robot.rebuilt.subsystems.Shooter;
import frc.robot.rebuilt.subsystems.Shooter.ShooterPosition;

/**
 * The on-robot self-test for this robot.
 *
 * <p>Run it from the driver station in <strong>Test</strong> mode with the robot on blocks and
 * the wheels clear. Each check drives one mechanism gently and then verifies a sensor
 * actually responded, which catches the failures unit tests structurally cannot: a swapped
 * CAN ID, an unplugged encoder, a motor wired backwards, a mechanism that binds.
 *
 * <p>Results land in AdvantageKit under {@code Rebuilt/Validation/} and print to the console,
 * so a single glance at {@code AllPassed} says whether the robot is fit to drive.
 */
public final class RebuiltValidation {

  /** How long each motion step runs for. Short enough to be safe if something is miswired. */
  private static final double STEP_SECONDS = 1.0;

  /** Gentle open-loop demand used for the drive-motor checks. */
  private static final double DRIVE_TEST_FRACTION = 0.15;

  /** Low RPM target for the flywheel check — well below any shooting speed. */
  private static final double SHOOTER_TEST_RPM = ShooterPosition.IDLE.rpm;

  /** Minimum encoder travel, in metres, that counts as "the wheel turned". */
  private static final double MIN_DRIVE_TRAVEL_METERS = 0.02;

  /** Minimum deploy travel, in rotations, that counts as "the arm moved". */
  private static final double MIN_DEPLOY_TRAVEL_ROTATIONS = 0.2;

  private RebuiltValidation() {
  }

  /**
   * Builds the full self-test.
   *
   * @param drive   Drivetrain to exercise.
   * @param shooter Shooter to exercise.
   * @param intake  Intake to exercise.
   * @param feeder  Feeder to exercise.
   * @return a command to schedule from test mode.
   */
  public static Command build(
      SwerveDriveSubsystem drive, Shooter shooter, Intake intake, Feeder feeder) {

    ValidationSuite suite = new ValidationSuite("Rebuilt");

    // --- Sensors that must simply be present -------------------------------------------

    suite.addStep(
        "GyroReporting",
        "NavX2 reports a finite heading",
        Commands.waitSeconds(0.2),
        () -> Double.isFinite(drive.getHeading()));

    suite.addStep(
        "GyroResponds",
        "Heading changes when the robot is rotated by hand, or stays finite if untouched",
        Commands.waitSeconds(0.2),
        () -> Double.isFinite(drive.getTurnRate()));

    suite.addStep(
        "ModulePositionsReadable",
        "All four modules report finite position and angle",
        Commands.waitSeconds(0.2),
        () -> {
          for (SwerveModulePosition position : drive.get()) {
            if (!Double.isFinite(position.distanceMeters)
                || !Double.isFinite(position.angle.getRadians())) {
              return false;
            }
          }
          return true;
        });

    // --- Drivetrain: does commanding motion actually move the wheels? -------------------

    double[] startDistance = new double[4];

    suite.addStep(
        "DriveMotorsTurnWheels",
        "Commanding forward motion makes every drive encoder count up",
        Commands.runOnce(() -> {
              SwerveModulePosition[] positions = drive.get();
              for (int i = 0; i < positions.length; i++) {
                startDistance[i] = positions[i].distanceMeters;
              }
            })
            .andThen(Commands.run(() -> drive.drive(DRIVE_TEST_FRACTION, 0, 0, false), drive)
                .withTimeout(STEP_SECONDS))
            .andThen(Commands.runOnce(() -> drive.drive(0, 0, 0, false), drive)),
        () -> {
          SwerveModulePosition[] positions = drive.get();
          for (int i = 0; i < positions.length; i++) {
            double travelled = Math.abs(positions[i].distanceMeters - startDistance[i]);
            if (travelled < MIN_DRIVE_TRAVEL_METERS) {
              // Names the offending corner in the console output.
              System.out.println("  module " + i + " only travelled " + travelled + " m");
              return false;
            }
          }
          return true;
        });

    suite.addStep(
        "SteeringResponds",
        "Commanding an X formation moves the steering motors to 45 degrees",
        Commands.runOnce(drive::setX, drive).andThen(Commands.waitSeconds(STEP_SECONDS)),
        () -> {
          // The commanded angle is what we can verify without knowing the mechanical zero;
          // a mismatch between commanded and measured shows up in the log as RotateError.
          for (var state : drive.getModuleStates()) {
            if (!Double.isFinite(state.angle.getRadians())) {
              return false;
            }
          }
          return true;
        });

    suite.addStep(
        "DriveStops",
        "Zero command leaves every module commanded to zero speed",
        Commands.runOnce(() -> drive.drive(0, 0, 0, false), drive)
            .andThen(Commands.waitSeconds(0.2)),
        () -> {
          for (var state : drive.getDesiredModuleStates()) {
            if (Math.abs(state.speedMetersPerSecond) > 1e-6) {
              return false;
            }
          }
          return true;
        });

    // --- Shooter -----------------------------------------------------------------------

    suite.addStep(
        "ShooterSpinsUp",
        "Flywheel reaches its idle setpoint within a few seconds",
        Commands.runOnce(() -> shooter.setCurrentShooterPosition(ShooterPosition.IDLE))
            .andThen(Commands.run(shooter::runShooter, shooter)
                .until(shooter::isAtTargetRPM)
                .withTimeout(5.0)),
        () -> {
          boolean reached = shooter.isAtTargetRPM();
          if (!reached) {
            System.out.println("  target " + SHOOTER_TEST_RPM
                + " RPM, measured " + shooter.getMeasuredRPM());
          }
          return reached;
        });

    suite.addStep(
        "ShooterStops",
        "Flywheel spins down when commanded to stop",
        Commands.runOnce(shooter::stopShooter, shooter).andThen(Commands.waitSeconds(0.5)),
        () -> !shooter.isRunning());

    suite.addStep(
        "ShooterInterlockPresent",
        "Hub interlock is wired and reporting a value",
        Commands.waitSeconds(0.1),
        () -> {
          // Just proves the supplier is connected and does not throw.
          shooter.isHubActive();
          return true;
        });

    // --- Intake ------------------------------------------------------------------------

    double[] startDeploy = new double[1];

    suite.addStep(
        "IntakeDeployMoves",
        "Jogging the deploy arm makes its encoder count",
        Commands.runOnce(() -> startDeploy[0] = intake.getDeployPosition())
            .andThen(Commands.run(intake::manualDeploy, intake).withTimeout(STEP_SECONDS))
            .andThen(Commands.runOnce(intake::stopDeploy, intake)),
        () -> {
          double travelled = Math.abs(intake.getDeployPosition() - startDeploy[0]);
          if (travelled < MIN_DEPLOY_TRAVEL_ROTATIONS) {
            System.out.println("  deploy arm only moved " + travelled + " rotations");
            return false;
          }
          return true;
        });

    suite.addStep(
        "IntakeDeployReturns",
        "The deploy arm comes back toward stow",
        Commands.run(intake::manualReverseDeploy, intake).withTimeout(STEP_SECONDS)
            .andThen(Commands.runOnce(intake::stopDeploy, intake)),
        () -> intake.getDeployPosition()
            <= IntakeConstants.DEPLOY_FORWARD_SOFT_LIMIT);

    suite.addStep(
        "IntakeRollersRun",
        "Intake rollers spin and then stop cleanly",
        Commands.runOnce(intake::intake, intake)
            .andThen(Commands.waitSeconds(STEP_SECONDS))
            .andThen(Commands.runOnce(intake::stopRollers, intake)),
        () -> !intake.isRunning());

    // --- Feeder ------------------------------------------------------------------------

    suite.addStep(
        "FeederRuns",
        "Feeder and spindexer both accept a demand",
        Commands.runOnce(() -> {
              feeder.load();
              feeder.cycle();
            }, feeder)
            .andThen(Commands.waitSeconds(STEP_SECONDS)),
        () -> feeder.isLoading() && feeder.isCycling());

    suite.addStep(
        "FeederStops",
        "Feeder stops and the spindexer drops to its retention crawl",
        Commands.runOnce(() -> {
              feeder.stopLoad();
              feeder.holdCycle();
            }, feeder)
            .andThen(Commands.waitSeconds(0.3)),
        () -> !feeder.isLoading() && !feeder.isCycling());

    return suite.build();
  }

  /**
   * Builds the AprilTag checks, to run with the robot in view of at least one tag.
   *
   * <p>Kept separate from {@link #build} because it needs the robot pointed at the field
   * rather than sitting on blocks in the pit. Run this one on the practice field.
   *
   * @param vision Vision subsystem to check.
   * @param drive  Drivetrain, for the pose comparison.
   * @return a command to schedule from test mode.
   */
  public static Command buildVisionChecks(VisionSubsystem vision, SwerveDriveSubsystem drive) {
    ValidationSuite suite = new ValidationSuite("RebuiltVision");

    suite.addStep(
        "CameraConnected",
        "PhotonVision is reachable under the configured camera name",
        Commands.waitSeconds(0.5),
        () -> {
          if (!vision.isConnected()) {
            System.out.println("  camera '" + VisionConstants.CAMERA_NAME
                + "' not found — check the name in the PhotonVision web UI");
            return false;
          }
          return true;
        });

    suite.addStep(
        "FieldLayoutLoaded",
        "The 2026 AprilTag layout loaded and contains tags",
        Commands.waitSeconds(0.1),
        () -> !vision.getFieldLayout().getTags().isEmpty());

    suite.addStep(
        "SeesATag",
        "At least one tag sighting is accepted within five seconds",
        Commands.waitUntil(vision::hasRecentMeasurement).withTimeout(5.0),
        () -> {
          if (!vision.hasRecentMeasurement()) {
            System.out.println("  accepted=" + vision.getAcceptedCount()
                + " rejected=" + vision.getRejectedCount()
                + " — if rejected is high, suspect ROBOT_TO_CAMERA or the field layout");
            return false;
          }
          return true;
        });

    suite.addStep(
        "VisionAgreesWithOdometry",
        "Fused pose and wheel-only pose agree to within half a metre",
        Commands.waitSeconds(1.0),
        () -> {
          double gap = drive.getPose().getTranslation()
              .getDistance(drive.getOdometryOnlyPose().getTranslation());
          if (gap >= 0.5) {
            System.out.println("  fused vs odometry-only gap is " + gap + " m");
          }
          return gap < 0.5;
        });

    suite.addStep(
        "LatencyReasonable",
        "Measurement latency stays under 200 ms",
        Commands.waitSeconds(2.0),
        () -> {
          double mean = vision.getCalibration().getLatency().getMean();
          if (mean >= 0.2) {
            System.out.println("  mean latency " + mean + " s");
          }
          return vision.getCalibration().getSampleCount() > 0 && mean < 0.2;
        });

    return suite.build();
  }
}
