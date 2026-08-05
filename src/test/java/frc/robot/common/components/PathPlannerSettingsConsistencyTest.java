package frc.robot.common.components;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.CommonConstants.DriveConstants;
import frc.robot.CommonConstants.ModuleConstants;
import frc.robot.CommonConstants.SwerveConstants;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

/**
 * Checks that PathPlanner's GUI settings and the robot's code constants describe the same robot.
 *
 * <p>Two sources of truth exist for the same physical facts. PathPlanner <em>plans</em> paths using
 * {@code src/main/deploy/pathplanner/settings.json}; the robot <em>executes</em> them using the
 * constants in {@code CommonConstants}. Where the two disagree, PathPlanner asks for motion the
 * drivetrain does not deliver, and the result is a systematic path error that looks like bad tuning
 * and cannot be tuned away.
 *
 * <p>PathPlanner does ship a checker for this — {@code RobotConfig.hasValidConfig()} compares code
 * against GUI settings — but it returns a bare boolean at runtime, on a robot, where nobody is
 * looking. This turns the same comparison into a build-time failure that names the specific
 * property.
 *
 * <p><b>Known divergences are pinned rather than hidden.</b> Two are recorded below as explicit
 * allowances with the decision each one needs. Pinning means they cannot silently grow, and
 * tightening the allowance after a measurement is a one-line change.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class PathPlannerSettingsConsistencyTest {

  /**
   * Known divergence in module location, in metres.
   *
   * <p><b>Zero, and it should stay zero.</b> Both sources now carry the CAD figure of 23.5 in
   * between module rotation axes, so ±0.29845 m.
   *
   * <p>The history is worth keeping, because both of the old values were wrong in the same way. The
   * code said 26.5 in and PathPlanner implied 27.01 in, and the 6.5 mm per side between them was
   * being chased as the discrepancy. CAD showed <b>neither described the module spacing</b> — 26.5 in
   * is the frame perimeter and the modules sit 1.5 in inboard of each rail. The real error was 12.8%,
   * not 1.9%, and it was hidden by two sources agreeing closely on the wrong quantity.
   *
   * <p><b>Decision needed:</b> measure the frame and make both match. 6.5 mm of module position
   * error skews the kinematics slightly, so commanded rotation and translation bleed into each
   * other — small, but it is exactly the sort of term that eats a 1 inch budget alongside the
   * others. Whichever figure is right, the other must change.
   */
  private static final double KNOWN_MODULE_LOCATION_DIVERGENCE = 0.0;

  /**
   * Known divergence in drive current limit, in amps.
   *
   * <p>PathPlanner assumes 60 A; {@code Configs.java} applies 50 A. PathPlanner therefore plans
   * accelerations assuming 20% more available torque than the drivetrain will actually produce, so
   * the robot falls behind its path during hard acceleration.
   *
   * <p><b>Decision needed:</b> pick one. Raising the code limit to 60 A gives more acceleration and
   * more brownout risk; lowering the PathPlanner figure to 50 A makes planned paths honest. This is
   * an electrical call, not a code one.
   */
  private static final double KNOWN_CURRENT_LIMIT_DIVERGENCE = 10.0;

  private RobotConfig guiConfig;

  @BeforeAll
  void loadGuiSettings() {
    // Reads src/main/deploy/pathplanner/settings.json. Present in this repo, so this genuinely
    // exercises the same parse the robot performs.
    guiConfig = org.junit.jupiter.api.Assertions.assertDoesNotThrow(
        RobotConfig::fromGUISettings,
        "PathPlanner settings.json could not be parsed. The robot now falls back to "
            + "PathPlannerConfig rather than failing to boot, but paths will follow worse.");
  }

  @Test
  @DisplayName("Both agree the drivetrain is holonomic")
  void holonomicAgrees() {
    assertTrue(guiConfig.isHolonomic,
        "PathPlanner is planning for a non-holonomic drivetrain, which would produce paths a "
            + "swerve chassis should not be asked to follow");
  }

  @Test
  @DisplayName("Both agree there are four modules")
  void moduleCountAgrees() {
    assertEquals(DriveConstants.kDriveKinematics.getModules().length, guiConfig.numModules);
  }

  @Test
  @DisplayName("Wheel radius agrees to within a millimetre")
  void wheelRadiusAgrees() {
    // PathPlanner records 0.038; the code derives 0.0381 from a nominal 3 in wheel. A 0.1 mm
    // difference is 0.26%, well inside anything that matters.
    assertEquals(ModuleConstants.kWheelDiameterMeters / 2.0,
        guiConfig.moduleConfig.wheelRadiusMeters, 0.001,
        "Wheel radius divergence scales every distance PathPlanner plans. Note that a "
            + "wheel-scale calibration changes the code value, so remember to update "
            + "settings.json too.");
  }

  @Test
  @DisplayName("Max drive speed agrees to within 0.1 m/s")
  void maxSpeedAgrees() {
    // 4.879 in settings against 4.8 in code: PathPlanner may plan marginally faster than the
    // drivetrain is allowed to go, and desaturation absorbs the difference.
    assertEquals(DriveConstants.kMaxSpeedMetersPerSecond,
        guiConfig.moduleConfig.maxDriveVelocityMPS, 0.1,
        "PathPlanner planning above the drivetrain's configured cap means the robot silently "
            + "falls behind its path at top speed");
  }

  @Test
  @DisplayName("Coefficient of friction agrees")
  void cofAgrees() {
    assertEquals(PathPlannerConfig.WHEEL_COF, guiConfig.moduleConfig.wheelCOF, 1e-6);
  }

  @Test
  @DisplayName("Mass and MOI in the fallback match the recorded measurements")
  void inertialPropertiesAgree() {
    // The fallback deliberately copies the team's measured figures rather than guessing, so these
    // must not drift apart.
    assertEquals(guiConfig.massKG, PathPlannerConfig.MASS_KG, 0.01,
        "The fallback config should carry the same measured mass as settings.json");
    assertEquals(guiConfig.MOI, PathPlannerConfig.MOI_KG_M2, 0.01,
        "The fallback config should carry the same measured MOI as settings.json");
  }

  @Test
  @DisplayName("Module locations diverge by no more than the pinned amount")
  void moduleLocationsWithinKnownDivergence() {
    Translation2d[] code = DriveConstants.kDriveKinematics.getModules();
    Translation2d[] gui = guiConfig.moduleLocations;

    assertEquals(code.length, gui.length);

    double worst = 0;
    int worstIndex = -1;
    for (int i = 0; i < code.length; i++) {
      // Compare by magnitude of offset: the two sources may order modules differently, but a
      // square chassis has the same offset magnitude at every corner.
      double difference = Math.abs(gui[i].getNorm() - code[i].getNorm());
      if (difference > worst) {
        worst = difference;
        worstIndex = i;
      }
    }

    // Deliberately a ceiling on the KNOWN divergence, not a blanket tolerance. If the frame is
    // measured and the constants reconciled, drop this to something tight.
    assertTrue(worst <= KNOWN_MODULE_LOCATION_DIVERGENCE * Math.sqrt(2) + 1e-6,
        "Module location divergence has grown beyond the pinned " + KNOWN_MODULE_LOCATION_DIVERGENCE
            + " m per axis: worst is " + worst + " m at module " + worstIndex
            + ". Measure the frame and make settings.json and DriveConstants agree.");
  }

  @Test
  @DisplayName("Drive current limit diverges by no more than the pinned amount")
  void currentLimitWithinKnownDivergence() {
    double difference =
        Math.abs(guiConfig.moduleConfig.driveCurrentLimit - SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    assertTrue(difference <= KNOWN_CURRENT_LIMIT_DIVERGENCE + 1e-6,
        "Drive current limit divergence has grown beyond the pinned "
            + KNOWN_CURRENT_LIMIT_DIVERGENCE + " A: PathPlanner has "
            + guiConfig.moduleConfig.driveCurrentLimit + " A, code applies "
            + SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT
            + " A. PathPlanner plans acceleration it cannot get.");
  }

  @Test
  @DisplayName("The pinned divergences are still real, so the allowances are not stale")
  void pinnedDivergencesStillExist() {
    // If someone reconciles the constants, these allowances become dead weight that would hide a
    // future regression. Failing here is the prompt to tighten them.
    Translation2d[] code = DriveConstants.kDriveKinematics.getModules();
    double moduleDifference = Math.abs(guiConfig.moduleLocations[0].getNorm() - code[0].getNorm());
    double currentDifference =
        Math.abs(guiConfig.moduleConfig.driveCurrentLimit - SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT);

    boolean stillDiverged = moduleDifference > 1e-4 || currentDifference > 1e-4;

    assertTrue(stillDiverged,
        "The code and PathPlanner settings now agree. Tighten "
            + "KNOWN_MODULE_LOCATION_DIVERGENCE and KNOWN_CURRENT_LIMIT_DIVERGENCE to zero so "
            + "this test starts guarding against regressions instead of tolerating them.");
  }
}
