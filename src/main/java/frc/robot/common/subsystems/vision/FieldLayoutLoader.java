package frc.robot.common.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * Loads the AprilTag field layout, preferring a wpical-calibrated one if it has been deployed.
 *
 * <p><b>wpical</b> measures where a field's tags actually are, rather than where the official layout
 * says they should be, and writes a corrected layout JSON. On a practice field assembled by humans
 * that correction is worth having: the welded and AndyMark 2026 layouts already differ by up to 3.6
 * cm, and a hand-built practice field can be further out than that.
 *
 * <p>So if {@value #CALIBRATED_LAYOUT_FILE} is present in the deploy directory it is used, and
 * otherwise the compiled-in layout is. Which one is active is <b>logged loudly and exposed as a
 * validation check</b>, for a reason that matters more than it sounds:
 *
 * <h2>A calibrated practice-field layout is wrong at competition</h2>
 *
 * <p>The whole point of calibrating is to describe <em>your</em> field, including its assembly
 * errors. An official event field does not have your field's errors — it has its own. Taking a
 * practice-calibrated layout to an event therefore makes vision worse than the official layout would
 * have been, by exactly the amount your practice field is out of spec.
 *
 * <p>This is a silent failure by nature: the file is in the deploy directory, nothing warns you, and
 * the poses are confidently wrong. Hence {@link #describeActive()}, the startup log line, and
 * {@code LayoutProvenance} being something the validation suite can assert on. <b>Delete the file
 * before an event, or accept a known error.</b>
 *
 * <h2>Absurd layouts are rejected rather than used</h2>
 *
 * <p>wpical's own documentation is explicit that it is not a silver bullet: it refines tags that are
 * already roughly right and cannot fix gross placement errors. A calibration that went wrong can
 * therefore emit a layout that is far off, and a layout far off is worse than the nominal one. Any
 * calibrated layout whose tags sit more than {@value #MAX_PLAUSIBLE_DEVIATION_METERS} m from the
 * official positions is refused, with the deviation reported.
 */
public final class FieldLayoutLoader {

    /** Filename looked for in the deploy directory. Matches what wpical writes. */
    public static final String CALIBRATED_LAYOUT_FILE = "calibrated_field_layout.json";

    /**
     * Largest per-tag deviation from the official layout that is believable, in metres.
     *
     * <p>0.30 m is generous for a practice field and still far below the scale of a calibration that
     * has genuinely failed — a mis-pinned reference tag or a bad camera model puts tags metres out,
     * not centimetres. Anything past this is rejected on the grounds that wpical only refines a layout
     * that is already roughly correct, so a large correction is not a correction.
     */
    public static final double MAX_PLAUSIBLE_DEVIATION_METERS = 0.30;

    /** Where the active layout came from. */
    public enum LayoutProvenance {
        /** The layout compiled into WPILib, from {@code VisionConstants.FIELD_LAYOUT}. */
        OFFICIAL,
        /** A wpical-calibrated layout found in the deploy directory and accepted. */
        CALIBRATED,
        /** A calibrated file was found but rejected; the official layout is in use instead. */
        CALIBRATED_REJECTED
    }

    /** The layout in use, where it came from, and how far it sits from official. */
    public record Result(AprilTagFieldLayout layout, LayoutProvenance provenance,
            double maxDeviationMeters, double meanDeviationMeters, int comparedTags,
            String detail) {

        /** @return true when a calibrated layout is active, which is wrong at an official event. */
        public boolean isCalibrated() {
            return provenance == LayoutProvenance.CALIBRATED;
        }

        /** @return a one-line summary for the log and the console. */
        public String describe() {
            return switch (provenance) {
                case OFFICIAL -> "field layout: OFFICIAL (" + VisionConstants.FIELD_LAYOUT + ")";
                case CALIBRATED -> String.format(
                        "field layout: CALIBRATED from %s — %d tags, max %.1f cm and mean %.1f cm "
                                + "from official. THIS IS YOUR PRACTICE FIELD: delete the file "
                                + "before an event or vision will be wrong there by this much.",
                        CALIBRATED_LAYOUT_FILE, comparedTags,
                        maxDeviationMeters * 100, meanDeviationMeters * 100);
                case CALIBRATED_REJECTED -> String.format(
                        "field layout: OFFICIAL — a calibrated file was found and REJECTED. %s",
                        detail);
            };
        }
    }

    private FieldLayoutLoader() {
    }

    /**
     * Loads from the real deploy directory.
     *
     * @return the layout and its provenance. Never throws; falls back to official on any problem.
     */
    public static Result load() {
        return load(Filesystem.getDeployDirectory());
    }

    /**
     * Loads from a given directory, so tests can supply one.
     *
     * @param deployDirectory Directory to look for the calibrated layout in.
     * @return the layout and its provenance.
     */
    public static Result load(File deployDirectory) {
        AprilTagFieldLayout official = AprilTagFieldLayout.loadField(VisionConstants.FIELD_LAYOUT);

        File candidate = new File(deployDirectory, CALIBRATED_LAYOUT_FILE);
        if (!candidate.isFile()) {
            // The normal case, and not a problem. No file means use what is compiled in.
            return official(official);
        }

        AprilTagFieldLayout calibrated;
        try {
            calibrated = new AprilTagFieldLayout(candidate.toPath());
        } catch (IOException | RuntimeException e) {
            // A malformed layout must not stop the robot booting. Vision degrading to the official
            // layout is a small loss; failing robotInit is a match.
            return rejected(official, 0, 0, 0,
                    "could not be parsed (" + e.getClass().getSimpleName() + ": "
                            + e.getMessage() + ")");
        }

        Deviation deviation = compare(calibrated, official);

        if (deviation.comparedTags == 0) {
            return rejected(official, 0, 0, 0,
                    "shares no tag IDs with the official layout, so it describes a different field");
        }

        if (deviation.max > MAX_PLAUSIBLE_DEVIATION_METERS) {
            return rejected(official, deviation.max, deviation.mean, deviation.comparedTags,
                    String.format("tag %d sits %.2f m from its official position, past the %.2f m "
                                    + "plausible limit — wpical refines layouts that are already "
                                    + "roughly right, so a correction this large means the "
                                    + "calibration failed rather than that the field is unusual",
                            deviation.worstTag, deviation.max, MAX_PLAUSIBLE_DEVIATION_METERS));
        }

        return new Result(calibrated, LayoutProvenance.CALIBRATED, deviation.max, deviation.mean,
                deviation.comparedTags, "accepted");
    }

    private static Result official(AprilTagFieldLayout layout) {
        return new Result(layout, LayoutProvenance.OFFICIAL, 0, 0, 0, "no calibrated layout found");
    }

    private static Result rejected(AprilTagFieldLayout official, double max, double mean,
            int compared, String detail) {
        return new Result(official, LayoutProvenance.CALIBRATED_REJECTED, max, mean, compared,
                detail);
    }

    /** Per-tag deviation between two layouts. */
    private record Deviation(double max, double mean, int comparedTags, int worstTag) {
    }

    /**
     * Compares two layouts tag by tag.
     *
     * <p>Only tags present in both are compared. A practice field with half the tags fitted is
     * normal and is not a reason to reject anything.
     */
    private static Deviation compare(AprilTagFieldLayout candidate, AprilTagFieldLayout official) {
        double max = 0;
        double sum = 0;
        int compared = 0;
        int worstTag = -1;

        for (AprilTag tag : candidate.getTags()) {
            Optional<Pose3d> officialPose = official.getTagPose(tag.ID);
            if (officialPose.isEmpty()) {
                continue;
            }

            double distance = tag.pose.getTranslation()
                    .getDistance(officialPose.get().getTranslation());

            sum += distance;
            compared++;
            if (distance > max) {
                max = distance;
                worstTag = tag.ID;
            }
        }

        return new Deviation(max, compared > 0 ? sum / compared : 0, compared, worstTag);
    }

    /**
     * Publishes the active layout's provenance, and prints it once.
     *
     * <p>Logged as a string rather than a boolean so it is legible in AdvantageScope without having
     * to remember which way round the flag goes.
     *
     * @param result The result of {@link #load()}.
     */
    public static void report(Result result) {
        System.out.println("[vision] " + result.describe());

        Logger.recordOutput("Vision/Layout/Provenance", result.provenance().name());
        Logger.recordOutput("Vision/Layout/IsCalibrated", result.isCalibrated());
        Logger.recordOutput("Vision/Layout/MaxDeviationMeters", result.maxDeviationMeters());
        Logger.recordOutput("Vision/Layout/MeanDeviationMeters", result.meanDeviationMeters());
        Logger.recordOutput("Vision/Layout/ComparedTags", result.comparedTags());
    }

    /** @return a description of the layout currently on disk, without loading it into the robot. */
    public static String describeActive() {
        return load().describe();
    }
}
