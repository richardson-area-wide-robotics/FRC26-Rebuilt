package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;

/**
 * Tests for heading unwrapping and rotation accumulation.
 *
 * <p>Small class, outsized consequences. Every manoeuvre in the calibration catalogue containing a
 * 270° turn is only correct because this distinguishes 270° one way from 90° the other, and the
 * gyro-scale sweep only works because several full revolutions accumulate instead of wrapping. An
 * error here would silently mis-run eight of the sixteen permutations and corrupt the measured gyro
 * scale factor at the same time.
 */
class RotationAccumulatorTest {

  @Nested
  @DisplayName("Shortest delta")
  class ShortestDelta {

    @Test
    @DisplayName("Small differences pass through unchanged")
    void smallDifferences() {
      assertEquals(10.0, RotationAccumulator.shortestDelta(0, 10), 1e-9);
      assertEquals(-10.0, RotationAccumulator.shortestDelta(0, -10), 1e-9);
      assertEquals(0.0, RotationAccumulator.shortestDelta(45, 45), 1e-9);
    }

    @Test
    @DisplayName("Wraps forward across +180")
    void wrapsForward() {
      // 179 to -179 is 2 degrees counter-clockwise, not 358 clockwise.
      assertEquals(2.0, RotationAccumulator.shortestDelta(179, -179), 1e-9);
    }

    @Test
    @DisplayName("Wraps backward across -180")
    void wrapsBackward() {
      assertEquals(-2.0, RotationAccumulator.shortestDelta(-179, 179), 1e-9);
    }

    @Test
    @DisplayName("Result always lies within a half turn")
    void alwaysWithinHalfTurn() {
      for (double from = -180; from <= 180; from += 7) {
        for (double to = -180; to <= 180; to += 7) {
          double delta = RotationAccumulator.shortestDelta(from, to);
          assertTrue(delta > -180.0001 && delta <= 180.0001,
              "Delta from " + from + " to " + to + " was " + delta
                  + ", which is more than half a turn and would break accumulation");
        }
      }
    }

    @Test
    @DisplayName("Handles unwrapped inputs beyond +/-180")
    void handlesUnwrappedInputs() {
      // Some sources report continuously rather than wrapped; the modulo copes.
      assertEquals(10.0, RotationAccumulator.shortestDelta(350, 360), 1e-9);
      assertEquals(-10.0, RotationAccumulator.shortestDelta(360, 350), 1e-9);
    }

    @Test
    @DisplayName("Exactly half a turn resolves consistently")
    void exactlyHalfTurn() {
      double delta = RotationAccumulator.shortestDelta(0, 180);
      assertEquals(180.0, Math.abs(delta), 1e-9,
          "A 180 degree step is genuinely ambiguous in direction, but its magnitude must be exact");
    }
  }

  @Nested
  @DisplayName("Accumulation")
  class Accumulation {

    @Test
    @DisplayName("A quarter turn left accumulates to +90")
    void quarterTurnLeft() {
      RotationAccumulator accumulator = new RotationAccumulator();
      accumulator.reset(0);

      for (double heading = 0; heading <= 90; heading += 5) {
        accumulator.update(heading);
      }

      assertEquals(90.0, accumulator.getAccumulatedDegrees(), 1e-9);
    }

    @Test
    @DisplayName("A quarter turn right accumulates to -90")
    void quarterTurnRight() {
      RotationAccumulator accumulator = new RotationAccumulator();
      accumulator.reset(0);

      for (double heading = 0; heading >= -90; heading -= 5) {
        accumulator.update(heading);
      }

      assertEquals(-90.0, accumulator.getAccumulatedDegrees(), 1e-9);
    }

    @Test
    @DisplayName("270 left accumulates to +270, not -90 — the case the turn command depends on")
    void twoSeventyLeftIsNotNinetyRight() {
      RotationAccumulator accumulator = new RotationAccumulator();
      accumulator.reset(0);

      // Rotate counter-clockwise through 270 degrees, with the reading wrapping at 180.
      for (double turned = 0; turned <= 270; turned += 5) {
        accumulator.update(wrap(turned));
      }

      assertEquals(270.0, accumulator.getAccumulatedDegrees(), 1e-9,
          "The final heading is identical to a 90 degree right turn, which is exactly why a "
              + "controller servoing to an absolute heading would take the short way round and "
              + "turn the wrong direction");
    }

    @Test
    @DisplayName("270 right accumulates to -270")
    void twoSeventyRight() {
      RotationAccumulator accumulator = new RotationAccumulator();
      accumulator.reset(0);

      for (double turned = 0; turned >= -270; turned -= 5) {
        accumulator.update(wrap(turned));
      }

      assertEquals(-270.0, accumulator.getAccumulatedDegrees(), 1e-9);
    }

    @Test
    @DisplayName("Three full revolutions accumulate to 1080, as the gyro sweep needs")
    void multipleRevolutions() {
      RotationAccumulator accumulator = new RotationAccumulator();
      accumulator.reset(0);

      for (double turned = 0; turned <= 1080; turned += 5) {
        accumulator.update(wrap(turned));
      }

      assertEquals(1080.0, accumulator.getAccumulatedDegrees(), 1e-9,
          "Without unwrapping this would read near zero, and the measured gyro scale factor "
              + "would be meaningless");
    }

    @Test
    @DisplayName("Reversing direction subtracts rather than adding magnitude")
    void reversalSubtracts() {
      RotationAccumulator accumulator = new RotationAccumulator();
      accumulator.reset(0);

      // Out to +90...
      for (double heading = 0; heading <= 90; heading += 5) {
        accumulator.update(heading);
      }
      // ...and back to 0.
      for (double heading = 90; heading >= 0; heading -= 5) {
        accumulator.update(heading);
      }

      assertEquals(0.0, accumulator.getAccumulatedDegrees(), 1e-9,
          "Turning out and back leaves net zero rotation, which is what makes closure error on "
              + "an out-and-back manoeuvre meaningful");
    }

    @Test
    @DisplayName("Reset clears the total and re-anchors")
    void resetClears() {
      RotationAccumulator accumulator = new RotationAccumulator();
      accumulator.reset(0);
      accumulator.update(45);
      assertEquals(45.0, accumulator.getAccumulatedDegrees(), 1e-9);

      accumulator.reset(45);
      assertEquals(0.0, accumulator.getAccumulatedDegrees(), 1e-9);
      assertEquals(45.0, accumulator.getLastHeadingDegrees(), 1e-9);

      accumulator.update(50);
      assertEquals(5.0, accumulator.getAccumulatedDegrees(), 1e-9);
    }

    @Test
    @DisplayName("First update without reset establishes the reference instead of jumping")
    void firstUpdateEstablishesReference() {
      RotationAccumulator accumulator = new RotationAccumulator();

      // A caller that forgets reset() should not record a spurious jump from zero to 137.
      assertEquals(0.0, accumulator.update(137), 1e-9);
      assertEquals(137.0, accumulator.getLastHeadingDegrees(), 1e-9);

      assertEquals(3.0, accumulator.update(140), 1e-9);
    }

    @Test
    @DisplayName("Accumulation is exact across many wraps, with no creeping drift")
    void noDriftAcrossManyWraps() {
      RotationAccumulator accumulator = new RotationAccumulator();
      accumulator.reset(0);

      // Ten revolutions in 1-degree steps: 3600 samples, every one crossing arbitrary wraps.
      for (int step = 1; step <= 3600; step++) {
        accumulator.update(wrap(step));
      }

      assertEquals(3600.0, accumulator.getAccumulatedDegrees(), 1e-6,
          "Floating point error must not accumulate over a long sweep");
    }

    @Test
    @DisplayName("Update returns the running total, so callers need not query separately")
    void updateReturnsTotal() {
      RotationAccumulator accumulator = new RotationAccumulator();
      accumulator.reset(0);
      assertEquals(30.0, accumulator.update(30), 1e-9);
      assertEquals(60.0, accumulator.update(60), 1e-9);
    }
  }

  /** Wraps a continuous angle to the ±180 range a gyro would report. */
  private static double wrap(double degrees) {
    double wrapped = degrees % 360.0;
    if (wrapped > 180) {
      wrapped -= 360;
    } else if (wrapped <= -180) {
      wrapped += 360;
    }
    return wrapped;
  }
}
