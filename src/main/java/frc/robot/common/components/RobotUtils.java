package frc.robot.common.components;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.experimental.UtilityClass;

import java.util.function.BooleanSupplier;

@UtilityClass
public class RobotUtils  {

  @Getter
  private RobotConfig robotConfig;

  private final PowerDistribution POWER_DISTRIBUTION = new PowerDistribution(20, ModuleType.kRev);
  private final Debouncer DEBOUNCER = new Debouncer(0.25, DebounceType.kRising);

  /**
   * Helper method to bind a control action to a command.
   *
   * @param control The button to bind to.
   * @param command The command to execute when that button is pressed.
   * @param stopCommand The command to execute when that button is *not* pressed
   *
   * @author Hudson Strub
   * @since 2025
   */
  public static void bindControl(Trigger control, Command command, Command stopCommand) {
    control.whileTrue(command).whileFalse(stopCommand);
  }

   /**
   * Load the robot config used for pathplanner.
   *
   * <p>Falls back to {@link PathPlannerConfig#fallbackConfig()} rather than throwing. This used
   * to raise a {@code RuntimeException} from {@code robotInit()}, so a missing or malformed
   * settings file in the deploy directory did not merely degrade autonomous — it stopped the
   * robot booting. It also discarded the original exception, leaving the console saying only
   * that loading had failed and never why.
   *
   * <p>A robot that boots with an approximate autonomous configuration is far more useful than
   * one that does not boot, and the real cause is now printed.
   *
   * @author Alan Trinh
   * @since 2025
   */
  public static void loadRobotConfig() {
    try {
      robotConfig = RobotConfig.fromGUISettings();
      usingFallbackConfig = false;
    } catch (Exception e) {
      // Print the actual cause; the previous version swallowed it entirely.
      System.err.println("PathPlanner GUI settings could not be read ("
          + e.getClass().getSimpleName() + ": " + e.getMessage()
          + "). Falling back to the config defined in PathPlannerConfig. Autonomous will run, "
          + "but check the MEASURE values there before trusting path accuracy.");
      robotConfig = PathPlannerConfig.fallbackConfig();
      usingFallbackConfig = true;
    }
  }

  /**
   * Whether the fallback configuration is in use.
   *
   * <p>Worth surfacing on the dashboard: paths will follow noticeably worse on the fallback, and
   * knowing that is the difference between debugging the right thing and the wrong one.
   */
  @Getter
  private boolean usingFallbackConfig;


  /**
   * Run a command for a given amount of time
   * 
   * @param seconds The amount of time to run commandDuring for, in seconds
   * @param commandDuring The command ran
   * @param commandAfter The command ran after the time has passed (Ex: Stop motor)
   * 
   *
   * @author Alan Trinh
   * @since 2025
   */
  public static Command timedCommand(double seconds, Command commandDuring, Command commandAfter){
    return Commands.deadline(Commands.waitSeconds(seconds), commandDuring).andThen(commandAfter);
  }


   /**
   * Move a motor to a relative position

   * @author Hudson Strub
   * @since 2025
   */
  public static void moveToPosition(SparkBase motor, double targetPosition) {
      // Set the target position using the built-in PID controller
      motor.getClosedLoopController().setSetpoint(targetPosition, ControlType.kPosition);
  }

  /**
   * Get the current power of a channel on the PDH
   *
   * @param channel the channel on the pdh to get the current of.
   *
   * @author Hudson Strub
   * @since 2026
   */
  public static double getPDHCurrent(int channel){
    return POWER_DISTRIBUTION.getCurrent(channel);
  }

  public static BooleanSupplier debounce(double currentDraw, double currentLimit) {
    return () -> DEBOUNCER.calculate(currentDraw > currentLimit);
  }
}