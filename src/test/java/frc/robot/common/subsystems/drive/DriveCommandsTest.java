package frc.robot.common.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.testutil.HalFixture;
import frc.robot.testutil.SharedSubsystems;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

/**
 * Tests for the two motion primitives every calibration manoeuvre is built from.
 *
 * <p>Motion is simulated by moving the pose estimate with {@code resetOdometry}, because the
 * simulator has no drivetrain physics — commanding a module does not make the robot travel. That is
 * enough to exercise what these commands actually decide: how they decompose displacement, when
 * they consider themselves finished, and that they leave the drivetrain stopped.
 *
 * <p>What this deliberately does <em>not</em> claim to cover: the turn command cannot be driven to
 * completion, because the simulated gyro never moves. Its unwrapping and accumulation logic — the
 * part that actually matters, and the part that makes a 270° turn different from a 90° one — is
 * covered exhaustively by {@code RotationAccumulatorTest} instead. The split is deliberate: the
 * maths is tested properly, and the command plumbing is checked for the faults a smoke test can
 * find.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DriveCommandsTest {

  private SwerveDriveSubsystem drive;

  @BeforeAll
  void setUpClass() {
    HalFixture.initialize();
    drive = SharedSubsystems.drive();
  }

  @BeforeEach
  void setUp() {
    HalFixture.enableTeleop(true);
    HalFixture.centreDriverSticks();
    HalFixture.resetSubsystem(drive);
    drive.drive(0, 0, 0, false);
    drive.resetOdometry(new Pose2d());
  }

  /** @return the largest absolute commanded module speed, in m/s. */
  private double maxCommandedSpeed() {
    double max = 0;
    for (SwerveModuleState state : drive.getDesiredModuleStates()) {
      max = Math.max(max, Math.abs(state.speedMetersPerSecond));
    }
    return max;
  }

  @Nested
  @DisplayName("Straight drive, closed loop")
  class StraightDrive {

    @Test
    @DisplayName("Requires the drivetrain, so it cannot fight the default command")
    void requiresDrivetrain() {
      DriveStraightClosedLoop command = new DriveStraightClosedLoop(drive, 1.0);
      assertTrue(command.getRequirements().contains(drive));
    }

    @Test
    @DisplayName("Not finished before it has moved anywhere")
    void notFinishedAtStart() {
      DriveStraightClosedLoop command = new DriveStraightClosedLoop(drive, 3.048);
      command.initialize();
      assertFalse(command.isFinished());
    }

    @Test
    @DisplayName("Commands motion while distance remains")
    void commandsMotionWhileRemaining() {
      DriveStraightClosedLoop command = new DriveStraightClosedLoop(drive, 3.048);
      command.initialize();
      command.execute();

      assertTrue(maxCommandedSpeed() > 0,
          "With 10 ft still to travel the drivetrain should be commanded to move");
    }

    @Test
    @DisplayName("Measures along-track distance from the fused pose, not the encoders")
    void measuresAlongTrackFromPose() {
      DriveStraightClosedLoop command = new DriveStraightClosedLoop(drive, 3.048);
      command.initialize();

      // Simulate having driven 2 m by moving the pose estimate. This is the whole point of the
      // command: distance comes from the tag-corrected pose, so wheel-scale error cannot affect
      // where it stops.
      drive.resetOdometry(new Pose2d(2.0, 0, new Rotation2d()));
      command.execute();

      assertEquals(2.0, command.getAlongTrackMeters(), 1e-6);
      assertFalse(command.isFinished(), "Still a metre short of the target");
    }

    @Test
    @DisplayName("Finishes once the fused pose reaches the target and settles")
    void finishesAtTarget() {
      DriveStraightClosedLoop command = new DriveStraightClosedLoop(drive, 3.048);
      command.initialize();

      drive.resetOdometry(new Pose2d(3.048, 0, new Rotation2d()));

      // The command requires several consecutive loops inside tolerance before declaring
      // completion, so that noise on the pose estimate cannot end the run early.
      for (int i = 0; i < 5; i++) {
        command.execute();
      }
      assertFalse(command.isFinished(), "Must not finish on the first loop inside tolerance");

      for (int i = 0; i < 10; i++) {
        command.execute();
      }
      assertTrue(command.isFinished(), "After settling it should be finished");
    }

    @Test
    @DisplayName("Stops commanding motion once inside tolerance")
    void stopsInsideTolerance() {
      DriveStraightClosedLoop command = new DriveStraightClosedLoop(drive, 3.048);
      command.initialize();

      drive.resetOdometry(new Pose2d(3.048, 0, new Rotation2d()));
      command.execute();

      assertTrue(maxCommandedSpeed() < 0.05,
          "Sitting on the target, along-track command should be zero; only cross-track and "
              + "heading corrections remain");
    }

    @Test
    @DisplayName("Leaves the drivetrain stopped when it ends")
    void stopsOnEnd() {
      DriveStraightClosedLoop command = new DriveStraightClosedLoop(drive, 3.048);
      command.initialize();
      command.execute();
      assertTrue(maxCommandedSpeed() > 0);

      command.end(false);
      assertTrue(maxCommandedSpeed() < 1e-6,
          "A command that ends without stopping the drivetrain leaves it running");
    }

    @Test
    @DisplayName("Reverse travel is measured as negative along-track")
    void reverseTravel() {
      DriveStraightClosedLoop command = new DriveStraightClosedLoop(drive, -3.048);
      command.initialize();

      drive.resetOdometry(new Pose2d(-1.5, 0, new Rotation2d()));
      command.execute();

      assertEquals(-1.5, command.getAlongTrackMeters(), 1e-6);
      assertFalse(command.isFinished());
    }

    @Test
    @DisplayName("Reverse run finishes at its negative target")
    void reverseFinishes() {
      DriveStraightClosedLoop command = new DriveStraightClosedLoop(drive, -3.048);
      command.initialize();

      drive.resetOdometry(new Pose2d(-3.048, 0, new Rotation2d()));
      for (int i = 0; i < 15; i++) {
        command.execute();
      }
      assertTrue(command.isFinished());
    }

    @Test
    @DisplayName("Travels along the heading it started on, not along field x")
    void travelsAlongStartingHeading() {
      // Start facing 90 degrees, so forward is field +y.
      drive.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(90)));

      DriveStraightClosedLoop command = new DriveStraightClosedLoop(drive, 2.0);
      command.initialize();

      // Move 2 m in field +y — directly forward from the robot's point of view.
      drive.resetOdometry(new Pose2d(0, 2.0, Rotation2d.fromDegrees(90)));
      command.execute();

      assertEquals(2.0, command.getAlongTrackMeters(), 1e-6,
          "Displacement must be projected onto the starting heading, not assumed to be field x");
    }

    @Test
    @DisplayName("Sideways drift does not count as progress")
    void lateralDriftIsNotProgress() {
      DriveStraightClosedLoop command = new DriveStraightClosedLoop(drive, 3.048);
      command.initialize();

      // Pure lateral displacement: no along-track progress at all.
      drive.resetOdometry(new Pose2d(0, 0.5, new Rotation2d()));
      command.execute();

      assertEquals(0.0, command.getAlongTrackMeters(), 1e-6,
          "Drifting sideways is cross-track error, and must not be mistaken for distance "
              + "travelled");
      assertFalse(command.isFinished());
    }

    @Test
    @DisplayName("Executing repeatedly never throws")
    void repeatedExecutionIsSafe() {
      DriveStraightClosedLoop command = new DriveStraightClosedLoop(drive, 3.048);
      assertDoesNotThrow(() -> {
        command.initialize();
        for (int i = 0; i < 200; i++) {
          command.execute();
        }
        command.end(true);
      });
    }
  }

  @Nested
  @DisplayName("Turn to relative heading")
  class Turn {

    @Test
    @DisplayName("Requires the drivetrain")
    void requiresDrivetrain() {
      TurnToRelativeHeading command = new TurnToRelativeHeading(drive, 90);
      assertTrue(command.getRequirements().contains(drive));
    }

    @Test
    @DisplayName("Remembers what it was asked to do, including 270")
    void remembersRequest() {
      assertEquals(90.0, new TurnToRelativeHeading(drive, 90).getRequestedDegrees(), 1e-9);
      assertEquals(270.0, new TurnToRelativeHeading(drive, 270).getRequestedDegrees(), 1e-9);
      assertEquals(-270.0, new TurnToRelativeHeading(drive, -270).getRequestedDegrees(), 1e-9);
    }

    @Test
    @DisplayName("Starts with zero accumulated rotation")
    void startsAtZero() {
      TurnToRelativeHeading command = new TurnToRelativeHeading(drive, 90);
      command.initialize();
      assertEquals(0.0, command.getAccumulatedDegrees(), 1e-9);
      assertFalse(command.isFinished());
    }

    @Test
    @DisplayName("Commands rotation, and only rotation, while the turn remains")
    void commandsRotationOnly() {
      TurnToRelativeHeading command = new TurnToRelativeHeading(drive, 90);
      command.initialize();
      command.execute();

      // Turning in place: every module is commanded to move, but the chassis translation should
      // be zero.
      var speeds = drive.getChassisSpeeds();
      assertTrue(Math.abs(speeds.vxMetersPerSecond) < 0.1,
          "A turn in place must not command forward translation");
      assertTrue(Math.abs(speeds.vyMetersPerSecond) < 0.1,
          "A turn in place must not command sideways translation");
    }

    @Test
    @DisplayName("A zero-degree turn is immediately within tolerance")
    void zeroTurnIsImmediatelySatisfied() {
      TurnToRelativeHeading command = new TurnToRelativeHeading(drive, 0);
      command.initialize();

      for (int i = 0; i < 15; i++) {
        command.execute();
      }
      assertTrue(command.isFinished(),
          "Nothing to do, so it should settle rather than spin forever");
    }

    @Test
    @DisplayName("Leaves the drivetrain stopped when it ends")
    void stopsOnEnd() {
      TurnToRelativeHeading command = new TurnToRelativeHeading(drive, 90);
      command.initialize();
      command.execute();

      command.end(false);
      assertTrue(maxCommandedSpeed() < 1e-6);
    }

    @Test
    @DisplayName("Executing repeatedly never throws, even when the target is unreachable")
    void repeatedExecutionIsSafe() {
      // The simulated gyro never moves, so this turn can never complete — which is a fair proxy
      // for a stalled or blocked robot on the field. It must not throw or misbehave.
      TurnToRelativeHeading command = new TurnToRelativeHeading(drive, 270);
      assertDoesNotThrow(() -> {
        command.initialize();
        for (int i = 0; i < 200; i++) {
          command.execute();
        }
        command.end(true);
      });
      assertFalse(command.isFinished(),
          "Without gyro motion the turn genuinely is incomplete, and it should say so rather "
              + "than declaring success");
    }
  }
}
