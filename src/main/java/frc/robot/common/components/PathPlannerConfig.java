package frc.robot.common.components;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.CommonConstants.DriveConstants;
import frc.robot.CommonConstants.ModuleConstants;
import frc.robot.CommonConstants.SwerveConstants;

/**
 * A robot configuration defined in code, used when PathPlanner's GUI settings cannot be read.
 *
 * <p>Two problems motivated this. {@code RobotConfig.fromGUISettings()} reads a file from the
 * deploy directory, and {@code RobotUtils.loadRobotConfig()} turned any failure into a
 * {@code RuntimeException} thrown from {@code robotInit()} — so a missing or malformed settings
 * file did not degrade autonomous, it prevented the robot from booting at all. It also discarded
 * the original exception, so the console said only "Failed to load robot config from GUI
 * settings" and never why.
 *
 * <p>A robot that boots with an approximate autonomous configuration is far more useful than one
 * that does not boot. And because this config can be built without a filesystem, it is also what
 * makes the PathPlanner wiring reachable from a test.
 *
 * <p>Everything here is derived from constants that are already known and verified, <b>except
 * three physical properties that must be measured</b> — marked MEASURE below. Wrong values here
 * degrade path-following accuracy; they do not stop the robot working.
 */
public final class PathPlannerConfig {

    /**
     * Robot mass in kilograms, including battery and bumpers.
     *
     * <p>Taken from {@code src/main/deploy/pathplanner/settings.json}, where the team has already
     * recorded 47.6272 kg — a real measurement, so it is used here rather than a guess. Keep the
     * two in step: PathPlanner uses mass to convert desired accelerations into wheel forces, so a
     * mismatch shows up as consistently over- or under-shooting the start of every path.
     */
    public static final double MASS_KG = 47.6272;

    /**
     * Moment of inertia about the vertical axis, in kg·m².
     *
     * <p>Also from the team's PathPlanner settings. For reference, treating the robot as a uniform
     * rectangular plate ({@code I = m(L² + W²)/12}) would predict about
     * {@code 47.6272 * (0.6731² + 0.6731²) / 12 = 3.60}, so the recorded 3.733 is consistent with
     * the frame and evidently deliberate rather than a default.
     */
    public static final double MOI_KG_M2 = 3.733;

    /**
     * Coefficient of friction between wheel and carpet.
     *
     * <p>1.0, matching the team's PathPlanner settings. A conservative figure for FRC tread on
     * competition carpet; well-worn tread on fresh carpet can exceed 1.2. Too high and PathPlanner
     * plans accelerations the wheels cannot deliver, and the robot slips through corners.
     */
    public static final double WHEEL_COF = 1.0;

    private PathPlannerConfig() {
    }

    /**
     * Builds the fallback configuration from this robot's known geometry and drivetrain.
     *
     * <p>Uses the same constants the drivetrain itself runs on, so wheel radius, gear reduction,
     * current limit and module positions cannot drift away from reality. The drive motor is a NEO
     * Vortex geared through the MAXSwerve L3 reduction, matching the hardware.
     *
     * @return a configuration usable by {@code AutoBuilder}.
     */
    public static RobotConfig fallbackConfig() {
        // The gearbox reduction belongs on the motor model, so PathPlanner sees torque at the
        // wheel rather than at the motor shaft.
        DCMotor driveMotor = DCMotor.getNeoVortex(1)
                .withReduction(ModuleConstants.kDrivingMotorReduction);

        ModuleConfig moduleConfig = new ModuleConfig(
                ModuleConstants.kWheelDiameterMeters / 2.0,
                DriveConstants.kMaxSpeedMetersPerSecond,
                WHEEL_COF,
                driveMotor,
                SwerveConstants.DRIVE_MOTOR_CURRENT_LIMIT,
                1);

        // Reuse the drivetrain's own kinematics so the module positions are guaranteed to match
        // the ones the robot actually drives with.
        Translation2d[] moduleLocations = DriveConstants.kDriveKinematics.getModules();

        return new RobotConfig(MASS_KG, MOI_KG_M2, moduleConfig, moduleLocations);
    }
}
