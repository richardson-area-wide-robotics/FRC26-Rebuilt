package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
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
import frc.robot.rebuilt.RebuiltConstants.LoadConstants;
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
  @DisplayName("Free speed is the drive motor's Vortex figure, and not integer divided")
  void freeSpeedMatchesTheActualDriveMotor() {
    // The drivetrain runs NEO Vortex on SPARK Flex: 6784 RPM per REV's datasheet.
    //
    // This previously read `5676 / 60`, integer-divided. 5676 RPM is the free speed of the
    // NEO 2.0 and NEO 1.1, and it is WPILib's MAXSwerve template default. This robot does carry
    // NEO 2.0s — the intake deploy and the spindexer, both SPARK MAX — so 5676 is a real figure
    // for a real motor here and only wrong because it names the wrong one. That is precisely why
    // it survived review. 19.5% of error went into the velocity feedforward.
    assertEquals(6784 / 60.0, ModuleConstants.kDrivingMotorFreeSpeedRps, 1e-9);
    assertNotEquals(94.0, ModuleConstants.kDrivingMotorFreeSpeedRps, "integer division");
    assertNotEquals(5676 / 60.0, ModuleConstants.kDrivingMotorFreeSpeedRps,
        "5676 RPM is the NEO 2.0 / NEO 1.1 free speed, not the Vortex's");
  }

  @Test
  @DisplayName("Every expected mechanism speed sits below its motor's free speed")
  void expectedSpeedsArePhysicallyPossible() {
    // Motors are derived from the controller type: SPARK Flex means Vortex, SPARK MAX means
    // NEO 2.0 unless it is swerve steering. So each mechanism has a known physical ceiling, and an
    // expected unloaded speed above it is impossible rather than merely optimistic.
    //
    // This matters because the jam threshold is a FRACTION of expected speed. Set expected too
    // high and a perfectly healthy mechanism reads as permanently slow, so the robot starts
    // clearing a jam that does not exist during a match. Too low is the safe direction: jams take
    // longer to catch, but nothing fires spuriously.
    double vortexFreeSpeedRpm = 6784;
    double neo20FreeSpeedRpm = 5676;

    assertTrue(LoadConstants.INTAKE_EXPECTED_RPM < vortexFreeSpeedRpm,
        "intake rollers are Vortex on SPARK Flex; " + LoadConstants.INTAKE_EXPECTED_RPM
            + " must be below " + vortexFreeSpeedRpm);

    assertTrue(LoadConstants.FEEDER_EXPECTED_RPM < vortexFreeSpeedRpm,
        "feeder is Vortex on SPARK Flex; " + LoadConstants.FEEDER_EXPECTED_RPM
            + " must be below " + vortexFreeSpeedRpm);

    assertTrue(LoadConstants.SPINDEXER_EXPECTED_RPM < neo20FreeSpeedRpm,
        "spindexer is NEO 2.0 on SPARK MAX, which is the SLOWER motor; "
            + LoadConstants.SPINDEXER_EXPECTED_RPM + " must be below " + neo20FreeSpeedRpm);

    // And the jam threshold has to leave room below a working mechanism, or clearing fires on
    // healthy operation.
    assertTrue(LoadConstants.JAM_SPEED_FRACTION > 0 && LoadConstants.JAM_SPEED_FRACTION < 1.0,
        "jam fraction must be a fraction of expected speed");
  }

  @Test
  @DisplayName("The shooter jam threshold is tighter than the roller one, as a flywheel needs")
  void shooterJamThresholdIsTighterThanRollers() {
    // A flywheel is speed-controlled and recovers within a few hundred ms of a shot, so it should
    // never sit far below setpoint for long. A roller genuinely bogs down when it bites a piece.
    // Using one threshold for both would either miss flywheel problems or jostle on every intake.
    assertTrue(LoadConstants.SHOOTER_JAM_SPEED_FRACTION > LoadConstants.JAM_SPEED_FRACTION,
        "flywheel threshold " + LoadConstants.SHOOTER_JAM_SPEED_FRACTION
            + " should be tighter (higher) than the roller threshold "
            + LoadConstants.JAM_SPEED_FRACTION);
  }

  @Test
  @DisplayName("Drive feedforward kV reflects the Vortex, so the max speed cap is conservative")
  void maxSpeedIsBelowPhysicalCapability() {
    // With the corrected free speed the drivetrain's physical top speed is ~5.7 m/s, so the
    // 4.8 m/s cap now genuinely is a chosen limit rather than an accidental one that
    // happened to match the wrong motor's free speed.
    assertTrue(ModuleConstants.kDriveWheelFreeSpeedRps > DriveConstants.kMaxSpeedMetersPerSecond,
        "Configured max speed should sit at or below the physical free speed; physical="
            + ModuleConstants.kDriveWheelFreeSpeedRps
            + " configured=" + DriveConstants.kMaxSpeedMetersPerSecond);
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
  @DisplayName("No climber CAN ID exists, because the mechanism was never built")
  void noClimberOnTheBus() {
    // The climber never made it onto the robot, so its subsystem and constants were removed
    // rather than left as dead code. If the mechanism is ever built, add its ID to CanIds
    // and extend canIdsAreUnique() to cover it.
    assertFalse(
        Arrays.stream(CanIds.class.getFields()).anyMatch(f -> f.getName().contains("CLIMBER")),
        "A climber CAN ID has appeared; add it to canIdsAreUnique() and update this test");
  }
}
