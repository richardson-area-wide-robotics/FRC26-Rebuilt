package frc.robot.common.subsystems.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.common.subsystems.vision.FieldLayoutLoader.LayoutProvenance;
import frc.robot.common.subsystems.vision.FieldLayoutLoader.Result;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Nested;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * Tests for loading a wpical-calibrated field layout.
 *
 * <p>The stakes are asymmetric, which is what shapes these. A calibrated layout that is slightly off
 * is better than the official one <em>on the field it was measured on</em>, and worse than the
 * official one anywhere else. A calibrated layout that is grossly off is worse everywhere. So the
 * loader has to accept small corrections, reject large ones, and above all never fail quietly.
 */
class FieldLayoutLoaderTest {

    @TempDir
    Path deployDir;

    /** The layout the robot ships with. */
    private static AprilTagFieldLayout official() {
        return AprilTagFieldLayout.loadField(VisionConstants.FIELD_LAYOUT);
    }

    /**
     * Writes a layout file with every official tag shifted by the given amount.
     *
     * @param shiftMeters How far to move each tag in +x.
     * @return the file written.
     */
    private File writeShiftedLayout(double shiftMeters) throws IOException {
        AprilTagFieldLayout base = official();
        List<AprilTag> shifted = new ArrayList<>();

        for (AprilTag tag : base.getTags()) {
            Pose3d moved = new Pose3d(
                    tag.pose.getTranslation().plus(new Translation3d(shiftMeters, 0, 0)),
                    tag.pose.getRotation());
            shifted.add(new AprilTag(tag.ID, moved));
        }

        AprilTagFieldLayout layout =
                new AprilTagFieldLayout(shifted, base.getFieldLength(), base.getFieldWidth());

        File file = deployDir.resolve(FieldLayoutLoader.CALIBRATED_LAYOUT_FILE).toFile();
        layout.serialize(file.toPath());
        return file;
    }

    @Nested
    @DisplayName("with no calibrated layout deployed")
    class NoFile {

        @Test
        @DisplayName("uses the official layout, and that is not an error")
        void fallsBackQuietly() {
            Result result = FieldLayoutLoader.load(deployDir.toFile());

            assertEquals(LayoutProvenance.OFFICIAL, result.provenance());
            assertFalse(result.isCalibrated());
            assertTrue(result.describe().contains("OFFICIAL"), result.describe());
        }

        @Test
        @DisplayName("still returns a usable layout with tags in it")
        void layoutIsUsable() {
            assertFalse(FieldLayoutLoader.load(deployDir.toFile()).layout().getTags().isEmpty());
        }
    }

    @Nested
    @DisplayName("with a plausibly calibrated layout")
    class PlausibleCalibration {

        private Result load() throws IOException {
            // 4 cm of correction — the scale of a real practice field assembly error, and larger
            // than the 3.6 cm between the welded and AndyMark official layouts.
            writeShiftedLayout(0.04);
            return FieldLayoutLoader.load(deployDir.toFile());
        }

        @Test
        @DisplayName("is accepted and used")
        void isAccepted() throws IOException {
            Result result = load();

            assertEquals(LayoutProvenance.CALIBRATED, result.provenance());
            assertTrue(result.isCalibrated());
        }

        @Test
        @DisplayName("reports how far it sits from official, in both max and mean")
        void reportsDeviation() throws IOException {
            Result result = load();

            assertEquals(0.04, result.maxDeviationMeters(), 1e-6);
            assertEquals(0.04, result.meanDeviationMeters(), 1e-6);
            assertTrue(result.comparedTags() > 0);
        }

        @Test
        @DisplayName("warns that it is the practice field and wrong at an event")
        void warnsAboutCompetition() throws IOException {
            String description = load().describe();

            // The failure this guards against is silent: the file sits in the deploy directory, the
            // robot uses it at an event, and every pose is wrong by the practice field's own error.
            assertTrue(description.contains("PRACTICE FIELD"), description);
            assertTrue(description.contains("before an event"), description);
        }
    }

    @Nested
    @DisplayName("with an implausible layout")
    class ImplausibleCalibration {

        @Test
        @DisplayName("is rejected in favour of the official layout")
        void isRejected() throws IOException {
            // 2 m out. wpical's own docs say it refines layouts that are already roughly right, so a
            // correction this size means the calibration failed — a mis-pinned reference tag or a
            // bad camera model — not that the field is unusual.
            writeShiftedLayout(2.0);

            Result result = FieldLayoutLoader.load(deployDir.toFile());

            assertEquals(LayoutProvenance.CALIBRATED_REJECTED, result.provenance());
            assertFalse(result.isCalibrated());
        }

        @Test
        @DisplayName("says which tag was worst and by how much")
        void explainsTheRejection() throws IOException {
            writeShiftedLayout(2.0);
            String description = FieldLayoutLoader.load(deployDir.toFile()).describe();

            assertTrue(description.contains("REJECTED"), description);
            assertTrue(description.contains("2.00 m"), description);
            assertTrue(description.contains("calibration failed"), description);
        }

        @Test
        @DisplayName("still hands back a working official layout rather than nothing")
        void stillUsable() throws IOException {
            writeShiftedLayout(2.0);
            Result result = FieldLayoutLoader.load(deployDir.toFile());

            assertEquals(official().getTags().size(), result.layout().getTags().size());
        }

        @Test
        @DisplayName("a correction just inside the limit is still accepted")
        void boundaryIsAccepted() throws IOException {
            writeShiftedLayout(FieldLayoutLoader.MAX_PLAUSIBLE_DEVIATION_METERS - 0.01);

            assertEquals(LayoutProvenance.CALIBRATED,
                    FieldLayoutLoader.load(deployDir.toFile()).provenance());
        }
    }

    @Nested
    @DisplayName("with a broken file")
    class BrokenFile {

        @Test
        @DisplayName("malformed JSON falls back rather than stopping the robot booting")
        void malformedJsonIsSurvivable() throws IOException {
            Files.writeString(deployDir.resolve(FieldLayoutLoader.CALIBRATED_LAYOUT_FILE),
                    "{ this is not a field layout");

            Result result = FieldLayoutLoader.load(deployDir.toFile());

            // Vision degrading to the official layout costs a little accuracy. Throwing out of
            // robotInit costs the match.
            assertEquals(LayoutProvenance.CALIBRATED_REJECTED, result.provenance());
            assertFalse(result.layout().getTags().isEmpty());
            assertTrue(result.describe().contains("REJECTED"), result.describe());
        }

        @Test
        @DisplayName("an empty file falls back too")
        void emptyFileIsSurvivable() throws IOException {
            Files.writeString(deployDir.resolve(FieldLayoutLoader.CALIBRATED_LAYOUT_FILE), "");

            Result result = FieldLayoutLoader.load(deployDir.toFile());
            assertFalse(result.isCalibrated());
            assertFalse(result.layout().getTags().isEmpty());
        }

        @Test
        @DisplayName("a layout describing a different field, with no shared tags, is rejected")
        void unrelatedLayoutIsRejected() throws IOException {
            // Tag IDs nothing like the real field's. Comparing against official finds no overlap, so
            // there is no evidence the layout is plausible and it must not be trusted.
            List<AprilTag> foreign = List.of(
                    new AprilTag(900, new Pose3d(1, 1, 1, new Rotation3d())),
                    new AprilTag(901, new Pose3d(2, 2, 1, new Rotation3d())));

            new AprilTagFieldLayout(foreign, 16.0, 8.0)
                    .serialize(deployDir.resolve(FieldLayoutLoader.CALIBRATED_LAYOUT_FILE));

            Result result = FieldLayoutLoader.load(deployDir.toFile());

            assertEquals(LayoutProvenance.CALIBRATED_REJECTED, result.provenance());
            assertTrue(result.describe().contains("different field"), result.describe());
        }
    }
}
