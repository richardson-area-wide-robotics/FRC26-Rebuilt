package frc.robot.common.components;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.config.RobotConfig;

import frc.robot.CommonConstants.DriveConstants;
import frc.robot.CommonConstants.ModuleConstants;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Tests for the code-defined PathPlanner configuration.
 *
 * <p>This closes the gap that {@code ContainerWiringTest} had to document rather than cover:
 * {@code RobotConfig.fromGUISettings()} needs a deploy directory, so the PathPlanner half of the
 * container wiring was previously unreachable from a test. A config that can be built without a
 * filesystem makes that path testable — and, more importantly, means a missing settings file no
 * longer stops the robot booting.
 */
class PathPlannerConfigTest {

  @Test
  @DisplayName("Fallback config builds without a filesystem")
  void buildsWithoutDeployDirectory() {
    // The whole point: no file is read, so this works in a test JVM and on a robot whose deploy
    // directory is missing or corrupt.
    assertDoesNotThrow(PathPlannerConfig::fallbackConfig);
    assertNotNull(PathPlannerConfig.fallbackConfig());
  }

  @Test
  @DisplayName("hasValidConfig compares against GUI settings, so it cannot be used here")
  void hasValidConfigIsAGuiComparison() {
    // Worth pinning down, because the name invites exactly the wrong assumption.
    // RobotConfig.hasValidConfig() calls fromGUISettings() internally and checks whether this
    // config AGREES with the settings file — it is a code-versus-GUI consistency check, not a
    // self-validity check. With no deploy directory the call throws internally, PathPlanner
    // raises its BAD_GUI_CONFIG alert, and the method returns false.
    //
    // So a false here says nothing about whether the fallback is usable, and the real
    // assurances are the field-level assertions in the rest of this class.
    RobotConfig config = PathPlannerConfig.fallbackConfig();
    assertDoesNotThrow(config::hasValidConfig,
        "It must at least not propagate the internal IOException");
  }

  @Test
  @DisplayName("Module locations match the drivetrain's own kinematics")
  void moduleLocationsMatchKinematics() {
    RobotConfig config = PathPlannerConfig.fallbackConfig();

    // Derived from the same kinematics object the robot drives with, so these cannot drift apart.
    assertEquals(4, config.numModules);
    assertEquals(DriveConstants.kDriveKinematics.getModules().length,
        config.moduleLocations.length);

    for (int i = 0; i < config.moduleLocations.length; i++) {
      assertEquals(DriveConstants.kDriveKinematics.getModules()[i].getX(),
          config.moduleLocations[i].getX(), 1e-9);
      assertEquals(DriveConstants.kDriveKinematics.getModules()[i].getY(),
          config.moduleLocations[i].getY(), 1e-9);
    }
  }

  @Test
  @DisplayName("Config is holonomic, as a swerve drivetrain must be")
  void isHolonomic() {
    assertTrue(PathPlannerConfig.fallbackConfig().isHolonomic,
        "A swerve chassis reported as non-holonomic would have PathPlanner planning "
            + "differential-drive paths");
  }

  @Test
  @DisplayName("Wheel radius is half the configured diameter")
  void wheelRadiusDerivesFromDiameter() {
    RobotConfig config = PathPlannerConfig.fallbackConfig();
    assertEquals(ModuleConstants.kWheelDiameterMeters / 2.0,
        config.moduleConfig.wheelRadiusMeters, 1e-9,
        "Radius must track the diameter constant, so a wheel-scale correction propagates here too");
  }

  @Test
  @DisplayName("Max drive velocity matches the drivetrain's configured cap")
  void maxVelocityMatchesDrivetrain() {
    RobotConfig config = PathPlannerConfig.fallbackConfig();
    assertEquals(DriveConstants.kMaxSpeedMetersPerSecond,
        config.moduleConfig.maxDriveVelocityMPS, 1e-9);
  }

  @Test
  @DisplayName("Mass and moment of inertia are positive and physically plausible")
  void inertialPropertiesArePlausible() {
    RobotConfig config = PathPlannerConfig.fallbackConfig();

    assertTrue(config.massKG > 20 && config.massKG < 80,
        "An FRC robot outside 20-80 kg suggests a units error, was " + config.massKG);
    assertTrue(config.MOI > 0, "Moment of inertia must be positive");

    // MOI is the team's measured figure, not a computed one. Sanity-check it against the
    // uniform-plate estimate I = m(L² + W²)/12: a real robot carries mass towards its edges, so
    // the true value should be in the same ballpark and not wildly below.
    double plateEstimate = config.massKG
        * (DriveConstants.kTrackWidth * DriveConstants.kTrackWidth
            + DriveConstants.kWheelBase * DriveConstants.kWheelBase) / 12.0;

    assertTrue(config.MOI > plateEstimate * 0.5 && config.MOI < plateEstimate * 3.0,
        "Measured MOI of " + config.MOI + " kg·m² is implausible against the uniform-plate "
            + "estimate of " + plateEstimate + " kg·m² — check the units in settings.json");
  }

  @Test
  @DisplayName("Friction force is positive, so PathPlanner can plan accelerations at all")
  void frictionIsPositive() {
    RobotConfig config = PathPlannerConfig.fallbackConfig();
    assertTrue(config.wheelFrictionForce > 0);
    assertTrue(config.maxTorqueFriction > 0);
  }

  @Test
  @DisplayName("Coefficient of friction is in a believable range for carpet")
  void cofIsPlausible() {
    assertTrue(PathPlannerConfig.WHEEL_COF > 0.5 && PathPlannerConfig.WHEEL_COF <= 1.5,
        "FRC tread on carpet sits around 1.0-1.2; outside 0.5-1.5 is not a real measurement");
  }

  @Test
  @DisplayName("Kinematics round-trip: chassis speeds survive conversion through the config")
  void kinematicsRoundTrip() {
    RobotConfig config = PathPlannerConfig.fallbackConfig();

    // A config with wrong module geometry would distort this conversion, which is exactly how a
    // bad config manifests during a path.
    var speeds = new edu.wpi.first.math.kinematics.ChassisSpeeds(1.0, 0.5, 0.25);
    var states = config.toSwerveModuleStates(speeds);
    var recovered = config.toChassisSpeeds(states);

    assertEquals(speeds.vxMetersPerSecond, recovered.vxMetersPerSecond, 1e-6);
    assertEquals(speeds.vyMetersPerSecond, recovered.vyMetersPerSecond, 1e-6);
    assertEquals(speeds.omegaRadiansPerSecond, recovered.omegaRadiansPerSecond, 1e-6);
  }
}
