package frc.robot.common.components.diagnostics;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

/**
 * A self-test you run on the real robot to prove each mechanism actually works.
 *
 * <p>Unit tests prove the code is right; this proves the <em>robot</em> is right. Each step
 * commands one mechanism for a fixed time, then checks a condition that can only be true if
 * the motor turned, the encoder counted, and the wiring is correct. Results are written to
 * AdvantageKit under {@code Validation/} and printed to the console.
 *
 * <p>Intended to be run from the driver station's <strong>Test</strong> mode while the robot
 * is on blocks. Steps use deliberately low speeds and short durations so a miswired
 * mechanism cannot hurt anything before the step ends.
 *
 * <p>A step that throws is recorded as a failure and does not abort the rest of the suite —
 * one broken sensor should not hide the state of everything else.
 */
public class ValidationSuite {

  /** One mechanism check: do something, then verify it had the expected effect. */
  private static final class Step {
    private final String name;
    private final String description;
    private final Command action;
    private final BooleanSupplier verify;

    private Boolean passed;
    private String detail = "";

    private Step(String name, String description, Command action, BooleanSupplier verify) {
      this.name = name;
      this.description = description;
      this.action = action;
      this.verify = verify;
    }
  }

  private final String suiteName;
  private final List<Step> steps = new ArrayList<>();

  /**
   * @param suiteName Name for this suite; used as the logging prefix.
   */
  public ValidationSuite(String suiteName) {
    this.suiteName = suiteName;
  }

  /**
   * Adds a check.
   *
   * @param name        Short stable name; becomes the log key.
   * @param description What a human should understand this check to mean.
   * @param action      Command that exercises the mechanism. Should finish on its own.
   * @param verify      Evaluated once immediately after {@code action} completes.
   * @return this suite, for chaining.
   */
  public ValidationSuite addStep(
      String name, String description, Command action, BooleanSupplier verify) {
    steps.add(new Step(name, description, action, verify));
    return this;
  }

  /**
   * Builds the command that runs every step in order and reports at the end.
   *
   * @return a command to schedule from test mode.
   */
  public Command build() {
    Command sequence = Commands.runOnce(this::reset);

    for (Step step : steps) {
      sequence = sequence
          .andThen(Commands.runOnce(() -> announce(step)))
          .andThen(step.action)
          .andThen(Commands.runOnce(() -> evaluate(step)));
    }

    return sequence.andThen(Commands.runOnce(this::report))
        .withName(suiteName + "Validation");
  }

  private void reset() {
    for (Step step : steps) {
      step.passed = null;
      step.detail = "";
    }
    Logger.recordOutput(suiteName + "/Validation/Running", true);
    System.out.println("=== " + suiteName + " validation starting: "
        + steps.size() + " checks ===");
  }

  private void announce(Step step) {
    System.out.println("[validate] " + step.name + " — " + step.description);
    Logger.recordOutput(suiteName + "/Validation/Current", step.name);
  }

  private void evaluate(Step step) {
    try {
      step.passed = step.verify.getAsBoolean();
    } catch (RuntimeException e) {
      step.passed = false;
      step.detail = e.getClass().getSimpleName() + ": " + e.getMessage();
    }

    String root = suiteName + "/Validation/" + step.name;
    Logger.recordOutput(root + "/Passed", step.passed);
    Logger.recordOutput(root + "/Detail", step.detail);

    System.out.println("[validate] " + step.name + ": "
        + (step.passed ? "PASS" : "FAIL " + step.detail));
  }

  private void report() {
    int passed = 0;
    int failed = 0;
    StringBuilder failures = new StringBuilder();

    for (Step step : steps) {
      if (Boolean.TRUE.equals(step.passed)) {
        passed++;
      } else {
        failed++;
        if (failures.length() > 0) {
          failures.append(", ");
        }
        failures.append(step.name);
      }
    }

    Logger.recordOutput(suiteName + "/Validation/Running", false);
    Logger.recordOutput(suiteName + "/Validation/Passed", passed);
    Logger.recordOutput(suiteName + "/Validation/Failed", failed);
    Logger.recordOutput(suiteName + "/Validation/AllPassed", failed == 0);
    Logger.recordOutput(suiteName + "/Validation/Failures", failures.toString());

    System.out.println("=== " + suiteName + " validation complete: "
        + passed + " passed, " + failed + " failed"
        + (failed == 0 ? "" : " (" + failures + ")") + " ===");
  }

  /** @return how many checks are registered. */
  public int size() {
    return steps.size();
  }
}
