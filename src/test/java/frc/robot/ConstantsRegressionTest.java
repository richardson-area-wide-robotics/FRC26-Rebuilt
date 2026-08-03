package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.util.Units;
import frc.robot.CommonConstants.DriveConstants;
import frc.robot.CommonConstants.HIDConstants;
import frc.robot.CommonConstants.ModuleConstants;
import frc.robot.CommonConstants.SwerveConstants;
import frc.robot.rebuilt.RebuiltConstants.CanIds;
import frc.robot.rebuilt.RebuiltConstants.IntakeConstants;
import frc.robot.rebuilt.RebuiltConstants.ShooterConstants;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Guards on the numbers the robot is built from.
 *
 * <p>Every assertion here corresponds to a defect that actually shipped, or to a physical
 * fact about the robot that must not silently change.
 */
class ConstantsRegressionTest {

  @Test
  @DisplayName("Free speed is not truncated by integer division")
  void freeSpeedIsNotIntegerDivided() {
    // Was `5676 / 60`, which is integer division and yields 94.0. The real value is 94.6,
    // and this feeds the drive velocity feedforward.
    assertEquals(94.6, ModuleConstants.kDrivingMotorFreeSpeedRps, 1e-9);
    assertNotEquals(94.0, ModuleConstants.kDrivingMotorFreeSpeedRps);
  }

  @Test
  @DisplayName("Drive feedforward derives from the corrected free speed")
  void driveWheelFreeSpeedIsConsistent() {
    double expected =
        (ModuleConstants.kDrivingMotorFreeSpeedRps * ModuleConstants.kWheelCircumferenceMeters)
            / ModuleConstants.kDrivingMotorReduction;
    assertEquals(expected, ModuleConstants.kDriveWheelFreeSpeedRps, 1e-9);
    assertTrue(ModuleConstants.kDriveWheelFreeSpeedRps > 0);
  }

  @Test
  @DisplayName("MAXSwerve L3 reduction matches the published gear train")
  void driveReductionMatchesHardware() {
    // 45T bevel, 22T first-stage spur, 15T bevel pinion, 14T driving pinion.
    assertEquals((45.0 * 22) / (14 * 15), ModuleConstants.kDrivingMotorReduction, 1e-9);
    assertEquals(14, ModuleConstants.kDrivingMotorPinionTeeth);
  }

  @Test
  @DisplayName("Controller deadband is usable")
  void deadbandIsSane() {
    // Was 0.6, which would discard 60% of stick travel. It was never actually applied,
    // so the value was harmless until the deadband got wired in — at which point it
    // would have made the robot feel broken.
    assertTrue(
        HIDConstants.CONTROLLER_DEADBAND > 0.0 && HIDConstants.CONTROLLER_DEADBAND <= 0.15,
        "Deadband should be a small positive fraction, was " + HIDConstants.CONTROLLER_DEADBAND);
  }

  @Test
  @DisplayName("Every CAN ID on the bus is unique")
  void canIdsAreUnique() {
    List<Integer> ids = List.of(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        CanIds.SHOOTER_LEADER,
        CanIds.SHOOTER_FOLLOWER,
        CanIds.INTAKE_DEPLOY,
        CanIds.INTAKE_ROLLER_1,
        CanIds.SPINDEXER,
        CanIds.INTAKE_ROLLER_2,
        CanIds.FEEDER);

    Set<Integer> unique = new HashSet<>(ids);
    assertEquals(ids.size(), unique.size(),
        "Duplicate CAN ID detected across swerve and superstructure: " + ids);
  }

  @Test
  @DisplayName("Chassis is square, matching the measured 26.5 inch frame")
  void chassisGeometryIsSquare() {
    assertEquals(Units.inchesToMeters(26.5), DriveConstants.kTrackWidth, 1e-9);
    assertEquals(Units.inchesToMeters(26.5), DriveConstants.kWheelBase, 1e-9);
  }

  @Test
  @DisplayName("Kinematics has exactly four modules")
  void kinematicsHasFourModules() {
    // toChassisSpeeds() is called with the measured module states; a mismatch in count
    // between kinematics and the module array throws at runtime.
    assertEquals(4, DriveConstants.kDriveKinematics.getModules().length);
  }

  @Test
  @DisplayName("Swerve current limits are the values actually applied to hardware")
  void currentLimitsMatchAppliedValues() {
    // These constants used to be dead: they declared 60 A while Configs.java applied 50 A.
    // Configs.java now reads them, so this test pins the real behaviour.
    assertEquals(50, SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    assertEquals(20, SwerveConstants.ROTATE_MOTOR_CURRENT_LIMIT);
    assertEquals(60, CommonConstants.SUPERSTRUCTURE_CURRENT_LIMIT);
  }

  @Test
  @DisplayName("Shooter presets are ordered by range and all positive")
  void shooterPresetsAreOrdered() {
    var idle = frc.robot.rebuilt.subsystems.Shooter.ShooterPosition.IDLE;
    var hub = frc.robot.rebuilt.subsystems.Shooter.ShooterPosition.HUB;
    var tower = frc.robot.rebuilt.subsystems.Shooter.ShooterPosition.TOWER;
    var trench = frc.robot.rebuilt.subsystems.Shooter.ShooterPosition.TRENCH;
    var corner = frc.robot.rebuilt.subsystems.Shooter.ShooterPosition.CORNER;

    assertTrue(idle.rpm > 0);
    assertTrue(idle.rpm < hub.rpm, "idle should be below the closest shot");
    assertTrue(hub.rpm < tower.rpm, "hub is the closest shot");
    assertTrue(tower.rpm < trench.rpm);
    assertTrue(trench.rpm < corner.rpm, "corner is the longest shot");
  }

  @Test
  @DisplayName("Operator trim cannot exceed the lowest preset")
  void operatorTrimIsBounded() {
    // A trim large enough to drive the commanded RPM negative would be a hazard.
    assertTrue(ShooterConstants.MAX_OPERATOR_TRIM_RPM > 0);
    assertTrue(
        ShooterConstants.MAX_OPERATOR_TRIM_RPM
            < frc.robot.rebuilt.subsystems.Shooter.ShooterPosition.IDLE.rpm,
        "Max trim must not be able to drive the idle preset to zero or below");
  }

  @Test
  @DisplayName("Intake soft limits bracket both deploy targets")
  void softLimitsBracketTargets() {
    assertTrue(IntakeConstants.DEPLOY_REVERSE_SOFT_LIMIT
        <= IntakeConstants.STOW_POSITION_ROTATIONS);
    assertTrue(IntakeConstants.DEPLOY_FORWARD_SOFT_LIMIT
        >= IntakeConstants.DEPLOY_POSITION_ROTATIONS);
  }

  @Test
  @DisplayName("Climber has no CAN ID, documenting that it is not on the robot")
  void climberIsUnassigned() {
    // If someone wires the climber up, this test should be updated deliberately — it is
    // here so the absence of a climb capability is explicit rather than accidental.
    assertEquals(-1, CanIds.CLIMBER_UNASSIGNED,
        "Climber has been given a CAN ID; instantiate it in RebuiltContainer and update "
            + "this test");
  }
}
