package frc.robot.common.components.diagnostics;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;

/**
 * Calibration values that persist across reboots and code deploys.
 *
 * <p>A robot's physical constants are not constant. Tread wears down, so the effective wheel
 * diameter shrinks. Gears bed in, belts stretch, bearings loosen. A wheel diameter measured in week
 * one is wrong by week six, and the usual result is someone re-tuning gains to compensate for a
 * geometry error.
 *
 * <p>This holds measured values in a JSON file in the roboRIO's home directory, which survives both
 * a reboot and a code deploy. Constants read through {@link CalibratedConstant} take the stored
 * value when one exists and the compiled-in value otherwise, so a fresh robot behaves exactly as
 * the source says while a broken-in one uses what was measured.
 *
 * <p><b>Nothing is ever written automatically.</b> Values are promoted only by an explicit call,
 * driven from Test mode with a human present. The reasoning is in
 * {@link #mayAutoAdopt(String)} — some quantities can be measured independently of themselves and
 * some cannot, and the ones that cannot must never close the loop on their own.
 *
 * <p>Every entry carries an audit trail: the value, when it was promoted, and how many samples it
 * came from. "Wheel diameter, updated three weeks ago from 340 samples" is a very different
 * statement from "wheel diameter, 0.0748".
 */
public class CalibrationStore {

    /** Filename inside the roboRIO home directory. */
    private static final String FILE_NAME = "calibration.json";

    /** JSON keys for each entry. */
    private static final String KEY_VALUE = "value";
    private static final String KEY_PROMOTED_AT = "promotedAt";
    private static final String KEY_SAMPLES = "samples";
    private static final String KEY_NOTE = "note";

    /** One stored calibration value and its provenance. */
    public record Entry(double value, String promotedAt, int samples, String note) {

        /** @return a one-line human summary, as printed by the report. */
        public String describe() {
            return String.format("%.6f  (promoted %s from %d samples%s)",
                    value, promotedAt, samples, note.isBlank() ? "" : ": " + note);
        }
    }

    private static CalibrationStore instance;

    private final Path path;
    private final Map<String, Entry> entries = new LinkedHashMap<>();
    private boolean loaded;
    private String loadError = "";

    /**
     * @param path Where to persist. Injectable so tests do not touch a real robot path.
     */
    public CalibrationStore(Path path) {
        this.path = path;
    }

    /**
     * The robot-wide store.
     *
     * <p>On a real robot this lives in the roboRIO home directory, which a code deploy does not
     * clear. In simulation it goes next to the project so a sim session does not silently inherit
     * or overwrite robot calibration.
     *
     * @return the singleton store.
     */
    public static synchronized CalibrationStore getInstance() {
        if (instance == null) {
            Path base = RobotBase.isReal()
                    ? Paths.get(Filesystem.getOperatingDirectory().getAbsolutePath())
                    : Paths.get(System.getProperty("user.dir"), "build");
            instance = new CalibrationStore(base.resolve(FILE_NAME));
            instance.load();
        }
        return instance;
    }

    /** Replaces the singleton. Tests only. */
    public static synchronized void setInstanceForTesting(CalibrationStore store) {
        instance = store;
    }

    /**
     * Reads the file if present.
     *
     * <p>A missing file is normal and not an error — it simply means nothing has been promoted yet.
     * A corrupt file is reported and then ignored, because refusing to boot over a malformed
     * calibration file would be a far worse outcome than running on compiled-in defaults.
     */
    public void load() {
        entries.clear();
        loadError = "";
        loaded = true;

        if (!Files.exists(path)) {
            System.out.println("[calibration] no store at " + path
                    + " — using compiled-in defaults for everything");
            return;
        }

        try {
            JSONObject root = new JSONObject(Files.readString(path));
            for (String name : root.keySet()) {
                JSONObject item = root.getJSONObject(name);
                entries.put(name, new Entry(
                        item.getDouble(KEY_VALUE),
                        item.optString(KEY_PROMOTED_AT, "unknown"),
                        item.optInt(KEY_SAMPLES, 0),
                        item.optString(KEY_NOTE, "")));
            }
            System.out.println("[calibration] loaded " + entries.size() + " values from " + path);
        } catch (IOException | RuntimeException e) {
            loadError = e.getClass().getSimpleName() + ": " + e.getMessage();
            entries.clear();
            System.err.println("[calibration] store at " + path + " could not be read ("
                    + loadError + "). Falling back to compiled-in defaults. The file has NOT been "
                    + "overwritten, so it can be inspected.");
        }
    }

    /**
     * Writes the current entries out.
     *
     * @return true on success. A failure is reported and does not throw: losing a calibration
     *     write is bad, but taking the robot down over it is worse.
     */
    public boolean save() {
        JSONObject root = new JSONObject();
        entries.forEach((name, entry) -> {
            JSONObject item = new JSONObject();
            item.put(KEY_VALUE, entry.value());
            item.put(KEY_PROMOTED_AT, entry.promotedAt());
            item.put(KEY_SAMPLES, entry.samples());
            item.put(KEY_NOTE, entry.note());
            root.put(name, item);
        });

        try {
            Files.createDirectories(path.getParent());
            Files.writeString(path, root.toString(2));
            System.out.println("[calibration] wrote " + entries.size() + " values to " + path);
            return true;
        } catch (IOException | RuntimeException e) {
            System.err.println("[calibration] could not write " + path + " ("
                    + e.getClass().getSimpleName() + ": " + e.getMessage() + ")");
            return false;
        }
    }

    /**
     * Records a measured value, replacing any previous one.
     *
     * <p>Does not write to disk; call {@link #save()} once after promoting a batch, so a set of
     * related values lands together rather than half-applied.
     *
     * @param name      Stable identifier, also the log key.
     * @param value     Measured value.
     * @param samples   How many observations it came from, for the audit trail.
     * @param timestamp When it was promoted. Passed in rather than read from a clock so this stays
     *                  testable and so the caller can use FMS match time if that is more useful.
     * @param note      Free text, e.g. which routine produced it.
     */
    public void promote(String name, double value, int samples, String timestamp, String note) {
        Entry previous = entries.get(name);
        entries.put(name, new Entry(value, timestamp, samples, note));

        if (previous == null) {
            System.out.printf("[calibration] %s promoted to %.6f (was compiled-in default)%n",
                    name, value);
        } else {
            double changePercent = previous.value() == 0
                    ? 0
                    : (value / previous.value() - 1) * 100;
            System.out.printf("[calibration] %s promoted %.6f -> %.6f (%+.2f%%)%n",
                    name, previous.value(), value, changePercent);
        }
    }

    /**
     * @param name Value name.
     * @return the stored entry, if one has been promoted.
     */
    public Optional<Entry> get(String name) {
        return Optional.ofNullable(entries.get(name));
    }

    /**
     * @param name            Value name.
     * @param compiledDefault Value from source.
     * @return the stored value if present, otherwise the compiled-in default.
     */
    public double getOrDefault(String name, double compiledDefault) {
        Entry entry = entries.get(name);
        return entry == null ? compiledDefault : entry.value();
    }

    /** Removes a stored value, so the compiled-in default applies again. */
    public void revert(String name) {
        if (entries.remove(name) != null) {
            System.out.println("[calibration] " + name + " reverted to its compiled-in default");
        }
    }

    /** Removes every stored value. */
    public void clear() {
        entries.clear();
    }

    /** @return an unmodifiable view of everything stored. */
    public Map<String, Entry> getEntries() {
        return Map.copyOf(entries);
    }

    /** @return true when the store has been read, whether or not it contained anything. */
    public boolean isLoaded() {
        return loaded;
    }

    /** @return the message from a failed load, or empty string if the load was clean. */
    public String getLoadError() {
        return loadError;
    }

    /** @return where this store persists to. */
    public Path getPath() {
        return path;
    }

    /**
     * Whether a value is safe to adopt without a human deciding.
     *
     * <p>The distinction is whether the quantity is measured <em>independently of itself</em>.
     *
     * <p>Vision measurement noise qualifies: its estimate comes from the spread of tag poses while
     * the robot is stationary, and adopting it changes only how much the estimator trusts vision.
     * The measurement does not depend on the value.
     *
     * <p>Wheel diameter does not qualify. It is measured by comparing odometry against a
     * vision-corrected pose — but wheel diameter also determines that odometry, and vision is what
     * corrects it. Adopting automatically closes a loop around the very quantity being estimated,
     * and a biased camera transform would walk the value steadily away from truth with nothing to
     * stop it. Same argument applies to gyro scale and to any control gain.
     *
     * @param name Value name.
     * @return true when automatic adoption is defensible.
     */
    public static boolean mayAutoAdopt(String name) {
        return name.startsWith("vision.noise.");
    }

    /** Publishes the store's state, so what the robot is actually running on is visible. */
    public void log() {
        Logger.recordOutput("Calibration/Store/Count", entries.size());
        Logger.recordOutput("Calibration/Store/Path", path.toString());
        Logger.recordOutput("Calibration/Store/LoadError", loadError);

        entries.forEach((name, entry) -> {
            String root = "Calibration/Store/" + name;
            Logger.recordOutput(root + "/Value", entry.value());
            Logger.recordOutput(root + "/PromotedAt", entry.promotedAt());
            Logger.recordOutput(root + "/Samples", entry.samples());
        });
    }

    /** Prints everything stored, with provenance. */
    public void printReport() {
        System.out.println("=====================================================");
        System.out.println(" CALIBRATION STORE — " + path);
        if (!loadError.isBlank()) {
            System.out.println(" LOAD FAILED: " + loadError);
        }
        if (entries.isEmpty()) {
            System.out.println(" empty — every constant is using its compiled-in default");
        } else {
            entries.forEach((name, entry) ->
                    System.out.printf("   %-32s %s%n", name, entry.describe()));
        }
        System.out.println("=====================================================");
    }
}
