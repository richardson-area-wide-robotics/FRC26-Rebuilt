package frc.robot.common.components.diagnostics;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * Tests for persistent calibration.
 *
 * <p>The failure modes that matter here are not arithmetic, they are filesystem: a store that does
 * not exist yet on a fresh robot, a store truncated by a power cut mid-write, a store hand-edited
 * into invalid JSON. Any of those must leave the robot running on compiled-in defaults rather than
 * refusing to boot, because a calibration file is not worth a match.
 */
class CalibrationStoreTest {

  @TempDir
  Path tempDir;

  private CalibrationStore storeAt(String filename) {
    return new CalibrationStore(tempDir.resolve(filename));
  }

  @Nested
  @DisplayName("Reading and writing")
  class Persistence {

    @Test
    @DisplayName("A missing store is normal, not an error")
    void missingStoreIsNormal() {
      CalibrationStore store = storeAt("absent.json");
      assertDoesNotThrow(store::load);

      assertTrue(store.isLoaded());
      assertTrue(store.getLoadError().isBlank(),
          "A fresh robot has no store yet; that is not a failure");
      assertTrue(store.getEntries().isEmpty());
    }

    @Test
    @DisplayName("Compiled-in defaults apply when nothing has been promoted")
    void defaultsApplyWhenEmpty() {
      CalibrationStore store = storeAt("empty.json");
      store.load();

      assertEquals(0.0762, store.getOrDefault("drive.wheelDiameter", 0.0762), 1e-9);
    }

    @Test
    @DisplayName("A promoted value survives a save and reload")
    void valueRoundTrips() {
      CalibrationStore store = storeAt("round-trip.json");
      store.load();
      store.promote("drive.wheelDiameter", 0.074676, 340, "2026-08-04", "10 ft acceptance run");
      assertTrue(store.save());

      // A new instance, as though the robot had been power-cycled or code redeployed.
      CalibrationStore reloaded = storeAt("round-trip.json");
      reloaded.load();

      assertEquals(0.074676, reloaded.getOrDefault("drive.wheelDiameter", 0.0762), 1e-9);

      var entry = reloaded.get("drive.wheelDiameter").orElseThrow();
      assertEquals(340, entry.samples());
      assertEquals("2026-08-04", entry.promotedAt());
      assertTrue(entry.note().contains("acceptance"),
          "Provenance is the point: a bare number cannot be judged, a number with a date and a "
              + "sample count can");
    }

    @Test
    @DisplayName("Promoting again replaces the previous value and keeps the newer provenance")
    void promotionReplaces() {
      CalibrationStore store = storeAt("replace.json");
      store.load();

      store.promote("drive.wheelDiameter", 0.0760, 100, "2026-07-01", "first");
      store.promote("drive.wheelDiameter", 0.0748, 500, "2026-08-04", "after break-in");

      var entry = store.get("drive.wheelDiameter").orElseThrow();
      assertEquals(0.0748, entry.value(), 1e-9);
      assertEquals(500, entry.samples());
      assertEquals("2026-08-04", entry.promotedAt());
    }

    @Test
    @DisplayName("Reverting restores the compiled-in default")
    void revertRestoresDefault() {
      CalibrationStore store = storeAt("revert.json");
      store.load();
      store.promote("drive.wheelDiameter", 0.070, 10, "2026-08-04", "suspect");

      store.revert("drive.wheelDiameter");

      assertEquals(0.0762, store.getOrDefault("drive.wheelDiameter", 0.0762), 1e-9,
          "Reverting is the escape hatch when a promoted value turns out to be wrong");
      assertTrue(store.get("drive.wheelDiameter").isEmpty());
    }

    @Test
    @DisplayName("Several values coexist independently")
    void multipleValues() {
      CalibrationStore store = storeAt("multi.json");
      store.load();

      store.promote("drive.wheelDiameter", 0.0748, 300, "2026-08-04", "");
      store.promote("vision.noise.xyStdDev", 0.032, 900, "2026-08-04", "");
      store.promote("gyro.scale", 0.994, 120, "2026-08-04", "");
      assertTrue(store.save());

      CalibrationStore reloaded = storeAt("multi.json");
      reloaded.load();

      assertEquals(3, reloaded.getEntries().size());
      assertEquals(0.0748, reloaded.getOrDefault("drive.wheelDiameter", 0), 1e-9);
      assertEquals(0.032, reloaded.getOrDefault("vision.noise.xyStdDev", 0), 1e-9);
      assertEquals(0.994, reloaded.getOrDefault("gyro.scale", 0), 1e-9);
    }
  }

  @Nested
  @DisplayName("Damaged stores")
  class Robustness {

    @Test
    @DisplayName("Invalid JSON falls back to defaults instead of throwing")
    void invalidJsonFallsBack() throws IOException {
      Path path = tempDir.resolve("corrupt.json");
      Files.writeString(path, "{ this is not json");

      CalibrationStore store = new CalibrationStore(path);
      assertDoesNotThrow(store::load,
          "A hand-edited or truncated store must not stop the robot booting");

      assertFalse(store.getLoadError().isBlank(), "The failure should be reported, not swallowed");
      assertTrue(store.getEntries().isEmpty());
      assertEquals(0.0762, store.getOrDefault("drive.wheelDiameter", 0.0762), 1e-9);
    }

    @Test
    @DisplayName("A truncated store, as a power cut mid-write would leave, is survivable")
    void truncatedStoreIsSurvivable() throws IOException {
      Path path = tempDir.resolve("truncated.json");
      Files.writeString(path, "{\n  \"drive.wheelDiameter\": {\n    \"value\": 0.074");

      CalibrationStore store = new CalibrationStore(path);
      assertDoesNotThrow(store::load);
      assertFalse(store.getLoadError().isBlank());
    }

    @Test
    @DisplayName("A corrupt store is left on disk for inspection, not overwritten")
    void corruptStoreIsPreserved() throws IOException {
      Path path = tempDir.resolve("preserve.json");
      String original = "{ broken";
      Files.writeString(path, original);

      new CalibrationStore(path).load();

      assertEquals(original, Files.readString(path),
          "Loading must not destroy the evidence — someone will want to see what went wrong");
    }

    @Test
    @DisplayName("An entry missing its provenance still yields its value")
    void partialEntryStillUsable() throws IOException {
      // A hand-written store, or one from an older schema, may lack the audit fields.
      Path path = tempDir.resolve("partial.json");
      Files.writeString(path, "{ \"drive.wheelDiameter\": { \"value\": 0.0750 } }");

      CalibrationStore store = new CalibrationStore(path);
      store.load();

      assertEquals(0.0750, store.getOrDefault("drive.wheelDiameter", 0.0762), 1e-9);
      var entry = store.get("drive.wheelDiameter").orElseThrow();
      assertEquals(0, entry.samples(), "Unknown provenance should read as zero samples");
      assertEquals("unknown", entry.promotedAt());
    }
  }

  @Nested
  @DisplayName("Automatic adoption policy")
  class AutoAdoptPolicy {

    @Test
    @DisplayName("Vision noise may be adopted automatically")
    void visionNoiseIsSafe() {
      // Measured from the spread of tag poses while stationary. Adopting it changes only how much
      // the estimator trusts vision; the measurement does not depend on the value.
      assertTrue(CalibrationStore.mayAutoAdopt("vision.noise.xyStdDev"));
      assertTrue(CalibrationStore.mayAutoAdopt("vision.noise.yawStdDev"));
    }

    @Test
    @DisplayName("Wheel diameter may not — it is measured against what it determines")
    void wheelDiameterNeedsAHuman() {
      // Wheel diameter is measured by comparing odometry against a vision-corrected pose, but it
      // also determines that odometry. Adopting automatically closes a loop around the quantity
      // being estimated, and a biased camera transform would walk it away from truth unchecked.
      assertFalse(CalibrationStore.mayAutoAdopt("drive.wheelDiameter"));
    }

    @Test
    @DisplayName("Gyro scale and control gains may not either")
    void otherValuesNeedAHuman() {
      assertFalse(CalibrationStore.mayAutoAdopt("gyro.scale"));
      assertFalse(CalibrationStore.mayAutoAdopt("drive.kV"));
      assertFalse(CalibrationStore.mayAutoAdopt("shooter.kP"));
      assertFalse(CalibrationStore.mayAutoAdopt("pathplanner.translationP"));
    }
  }
}
