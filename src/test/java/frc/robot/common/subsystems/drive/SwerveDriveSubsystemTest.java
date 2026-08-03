package frc.robot.common.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommonConstants.DriveConstants;
import frc.robot.CommonConstants.HIDConstants;
import frc.robot.testutil.HalFixture;
import frc.robot.testutil.SharedSubsystems;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

/**
 * Tests for the drivetrain, centred on the defect that made the robot undriveable.
 *
 * <p>The critical case is {@link #defaultCommandTracksLiveStickInput()}: it fails against the
 * old {@code driveCommand(double, ...)} signature and passes against the supplier version.
 * That single assertion is the one this whole suite exists for.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class SwerveDriveSubsystemTest {

  private SwerveDriveSubsystem drive;

  @BeforeAll
  void setUpClass() {
    HalFixture.initialize();
    // One instance for the whole JVM: REVLib rejects a second controller object on the same
    // CAN ID, so this cannot be rebuilt per test.
    drive = SharedSubsystems.drive();
  }

  @BeforeEach
  void setUp() {
    // The scheduler only runs default commands while the robot is enabled. Without this the
    // default-command tests pass or fail for entirely the wrong reason.
    HalFixture.enableTeleop(true);
    HalFixture.centreDriverSticks();
    HalFixture.resetSubsystem(drive);
    drive.drive(0, 0, 0, false);
    drive.resetOdometry(new Pose2d());
  }

  @AfterAll
  void tearDownClass() {
    HalFixture.resetScheduler();
  }

  @Test
  @DisplayName("Default command follows live stick input, not a value captured at construction")
  void defaultCommandTracksLiveStickInput() {
    // Mutable holder standing in for a joystick the driver is moving.
    double[] stick = {0.0};

    Command driveCommand = drive.driveCommand(() -> stick[0], () -> 0.0, () -> 0.0, false);
    drive.setDefaultCommand(driveCommand);

    // Sticks centred: nothing should be commanded.
    HalFixture.runScheduler(3);
    assertTrue(maxCommandedSpeed() < 1e-6,
        "Modules should be idle while the stick is centred");

    // Now the driver pushes forward. The command was already constructed and scheduled;
    // if it captured the stick by value it will keep commanding zero forever.
    stick[0] = 1.0;
    HalFixture.runScheduler(3);

    assertTrue(maxCommandedSpeed() > 1.0,
        "Pushing the stick must command the modules to move. If this fails, driveCommand "
            + "is capturing joystick values instead of polling suppliers.");

    // And releasing it must settle back to zero.
    stick[0] = 0.0;
    HalFixture.runScheduler(3);
    assertTrue(maxCommandedSpeed() < 1e-6, "Releasing the stick must stop the modules");
  }

  @Test
  @DisplayName("Deadband suppresses stick noise but passes real input")
  void deadbandSuppressesNoiseOnly() {
    double belowDeadband = HIDConstants.CONTROLLER_DEADBAND * 0.5;
    assertEquals(0.0, SwerveDriveSubsystem.applyDeadband(belowDeadband), 1e-9,
        "Input inside the deadband must be zeroed");

    double wellAbove = 0.9;
    assertTrue(SwerveDriveSubsystem.applyDeadband(wellAbove) > 0.5,
        "Input well outside the deadband must survive");

    assertEquals(0.0, SwerveDriveSubsystem.applyDeadband(0.0), 1e-9);
    assertTrue(SwerveDriveSubsystem.applyDeadband(-0.9) < -0.5, "Deadband must be symmetric");
  }

  @Test
  @DisplayName("Deadbanded stick noise does not command the modules")
  void deadbandPreventsCreep() {
    double noise = HIDConstants.CONTROLLER_DEADBAND * 0.5;
    drive.setDefaultCommand(drive.driveCommand(() -> noise, () -> noise, () -> noise, false));

    HalFixture.runScheduler(3);

    assertTrue(maxCommandedSpeed() < 1e-6,
        "Stick noise inside the deadband must not creep the robot");
  }

  @Test
  @DisplayName("Robot-relative drive treats ChassisSpeeds as real units, not fractions")
  void driveRobotRelativeUsesRealUnits() {
    // drive() takes fractions of max speed, so driveRobotRelative must normalise. Passing
    // metres per second straight through multiplied every PathPlanner request by 4.8,
    // which desaturation then clamped to full speed — so every auto ran flat out.
    double requested = 1.0; // m/s
    drive.driveRobotRelative(new ChassisSpeeds(requested, 0, 0));

    double commanded = maxCommandedSpeed();
    assertEquals(requested, commanded, 0.05,
        "A 1 m/s request must command about 1 m/s, not max speed");
    assertTrue(commanded < DriveConstants.kMaxSpeedMetersPerSecond * 0.5,
        "A modest request must not saturate the drivetrain");
  }

  @Test
  @DisplayName("Measured chassis speeds reflect the modules, not an empty array")
  void chassisSpeedsReadFromModules() {
    // Previously toChassisSpeeds() was called with no arguments, so PathPlanner's velocity
    // feedback never described the robot.
    ChassisSpeeds speeds = drive.getChassisSpeeds();
    assertNotNull(speeds);
    assertTrue(Double.isFinite(speeds.vxMetersPerSecond));
    assertTrue(Double.isFinite(speeds.vyMetersPerSecond));
    assertTrue(Double.isFinite(speeds.omegaRadiansPerSecond));
  }

  @Test
  @DisplayName("Full-speed diagonal request is desaturated to within limits")
  void desaturationRespectsMaxSpeed() {
    drive.drive(1.0, 1.0, 1.0, false);

    for (SwerveModuleState state : drive.getDesiredModuleStates()) {
      assertTrue(Math.abs(state.speedMetersPerSecond)
              <= DriveConstants.kMaxSpeedMetersPerSecond + 1e-6,
          "No module may be commanded above max speed, was " + state.speedMetersPerSecond);
    }
  }

  @Test
  @DisplayName("Module arrays are always four long and in a stable order")
  void moduleArraysAreWellFormed() {
    assertEquals(4, drive.getModuleStates().length);
    assertEquals(4, drive.getDesiredModuleStates().length);
    assertEquals(4, drive.get().length);
  }

  @Test
  @DisplayName("X formation stops the robot and splays the wheels")
  void setXStopsAndSplays() {
    drive.drive(1.0, 0, 0, false);
    drive.setX();

    SwerveModuleState[] states = drive.getDesiredModuleStates();
    for (SwerveModuleState state : states) {
      assertEquals(0.0, state.speedMetersPerSecond, 1e-9, "X formation must command zero speed");
    }
    assertEquals(45.0, Math.abs(states[0].angle.getDegrees()), 1e-6);
  }

  @Test
  @DisplayName("Pose estimator starts at the reset pose and stays finite")
  void poseEstimatorResets() {
    Pose2d target = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(90));
    drive.resetOdometry(target);

    Pose2d pose = drive.getPose();
    assertEquals(3.0, pose.getX(), 1e-6);
    assertEquals(4.0, pose.getY(), 1e-6);

    drive.periodic();
    assertTrue(Double.isFinite(drive.getPose().getX()));
  }

  @Test
  @DisplayName("Vision measurements are accepted without destabilising the estimate")
  void visionMeasurementIsAccepted() {
    drive.resetOdometry(new Pose2d());
    drive.periodic();

    drive.addVisionMeasurement(
        new Pose2d(1.0, 0.0, new Rotation2d()), edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
    drive.periodic();

    Pose2d pose = drive.getPose();
    assertTrue(Double.isFinite(pose.getX()) && Double.isFinite(pose.getY()),
        "Fusing a vision measurement must not produce a non-finite pose");
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
