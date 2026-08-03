package frc.robot.common.components.diagnostics;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.components.diagnostics.CalibrationManeuvers.Leg;
import frc.robot.common.components.diagnostics.CalibrationManeuvers.LegType;
import frc.robot.common.components.diagnostics.CalibrationManeuvers.Maneuver;
import frc.robot.common.subsystems.drive.DriveStraightClosedLoop;
import frc.robot.common.subsystems.drive.SwerveDriveSubsystem;
import frc.robot.common.subsystems.drive.TurnToRelativeHeading;
import org.littletonrobotics.junction.Logger;

/**
 * Runs calibration manoeuvres and scores each one against where it should have finished.
 *
 * <p>Every manoeuvre has an analytically known finishing pose, so the difference between that
 * and the AprilTag-corrected pose is the error — decomposed into along-track, cross-track and
 * heading so the cause is identifiable rather than just the magnitude.
 *
 * <p>For manoeuvres that return to their start, closure error is reported as well. That figure
 * is the most useful single number for a loop: it needs no absolute reference at all, only that
 * the robot came back to where it began.
 */
public class ManeuverRunner {

    /** Result of running one manoeuvre. */
    public record ManeuverResult(
            String name,
            double totalDistanceMeters,
            double alongTrackErrorMeters,
            double crossTrackErrorMeters,
            double headingErrorDegrees,
            double closureErrorMeters,
            boolean returnsToStart) {

        /** @return total positional error against the expected finishing pose, in metres. */
        public double totalErrorMeters() {
            return Math.hypot(alongTrackErrorMeters, crossTrackErrorMeters);
        }

        /**
         * Error as a fraction of distance driven, which is the fair way to compare a 5 ft
         * manoeuvre with a 40 ft one.
         *
         * @return error per metre driven, or 0 if nothing was driven.
         */
        public double errorPerMeter() {
            return totalDistanceMeters <= 0 ? 0 : totalErrorMeters() / totalDistanceMeters;
        }

        /**
         * @param toleranceMeters Allowed error.
         * @return true when within tolerance. Uses closure error for loops, since that is the
         *     figure a loop is actually judged on.
         */
        public boolean passed(double toleranceMeters) {
            return returnsToStart
                    ? closureErrorMeters <= toleranceMeters
                    : totalErrorMeters() <= toleranceMeters;
        }
    }

    private final SwerveDriveSubsystem drive;
    private final List<ManeuverResult> results = new ArrayList<>();

    /**
     * @param drive Drivetrain to command.
     */
    public ManeuverRunner(SwerveDriveSubsystem drive) {
        this.drive = drive;
    }

    /**
     * Builds a command that runs one manoeuvre and records its result.
     *
     * <p>Each driving leg uses the closed-loop straight drive, so the manoeuvre measures how
     * well the robot can hold and terminate a line using tags and gyro — which is the mode it
     * would actually compete in. Running the same manoeuvre with correction disabled measures
     * raw odometry instead.
     *
     * @param maneuver Manoeuvre to run.
     * @return a command that performs it and scores it.
     */
    public Command run(Maneuver maneuver) {
        Pose2d[] startPose = new Pose2d[1];

        Command sequence = Commands.runOnce(() -> {
            startPose[0] = drive.getPose();
            System.out.println("[maneuver] " + maneuver.name() + " — " + maneuver.description());
        });

        for (Leg leg : maneuver.legs()) {
            if (leg.type() == LegType.DRIVE) {
                sequence = sequence.andThen(
                        new DriveStraightClosedLoop(drive, leg.value()).withTimeout(15.0));
            } else {
                sequence = sequence.andThen(
                        new TurnToRelativeHeading(drive, leg.value()).withTimeout(8.0));
            }
            // Settle between legs so the pose estimate is current before the next one starts.
            sequence = sequence.andThen(Commands.waitSeconds(0.4));
        }

        return sequence
                .andThen(Commands.waitSeconds(1.0))
                .andThen(Commands.runOnce(() -> score(maneuver, startPose[0])))
                .withName("Maneuver_" + maneuver.name());
    }

    /**
     * Builds a command that runs a whole list of manoeuvres in order.
     *
     * @param maneuvers Manoeuvres to run.
     * @return a command performing all of them, then printing a summary.
     */
    public Command runAll(List<Maneuver> maneuvers) {
        Command sequence = Commands.runOnce(() -> {
            results.clear();
            System.out.printf("[maneuver] running %d manoeuvres, %.1f m of driving total%n",
                    maneuvers.size(), CalibrationManeuvers.totalDistanceMeters(maneuvers));
        });

        for (Maneuver maneuver : maneuvers) {
            sequence = sequence.andThen(run(maneuver));
        }

        return sequence.andThen(Commands.runOnce(this::printSummary))
                .withName("ManeuverSuite");
    }

    private void score(Maneuver maneuver, Pose2d startPose) {
        Pose2d expected = CalibrationManeuvers.expectedPose(startPose, maneuver.legs());
        Pose2d actual = drive.getPose();

        Translation2d error = actual.getTranslation().minus(expected.getTranslation());

        // Decompose against the expected final heading, so along and cross mean the same thing
        // they do for a straight run.
        double heading = expected.getRotation().getRadians();
        double along = error.getX() * Math.cos(heading) + error.getY() * Math.sin(heading);
        double cross = -error.getX() * Math.sin(heading) + error.getY() * Math.cos(heading);

        double headingError = MathUtil.inputModulus(
                actual.getRotation().getDegrees() - expected.getRotation().getDegrees(),
                -180, 180);

        double closure = maneuver.returnsToStart()
                ? actual.getTranslation().getDistance(startPose.getTranslation())
                : 0;

        ManeuverResult result = new ManeuverResult(
                maneuver.name(),
                maneuver.totalDistanceMeters(),
                along,
                cross,
                headingError,
                closure,
                maneuver.returnsToStart());
        results.add(result);

        String root = "Calibration/Maneuvers/" + maneuver.name();
        Logger.recordOutput(root + "/AlongErrorMm", along * 1000);
        Logger.recordOutput(root + "/CrossErrorMm", cross * 1000);
        Logger.recordOutput(root + "/TotalErrorMm", result.totalErrorMeters() * 1000);
        Logger.recordOutput(root + "/HeadingErrorDeg", headingError);
        Logger.recordOutput(root + "/ClosureErrorMm", closure * 1000);
        Logger.recordOutput(root + "/ErrorPerMeterMm", result.errorPerMeter() * 1000);
        Logger.recordOutput(root + "/ExpectedPose", expected);
        Logger.recordOutput(root + "/ActualPose", actual);

        if (maneuver.returnsToStart()) {
            System.out.printf(
                    "[maneuver] %-26s closure %6.1f mm  heading %+6.2f deg  (%.1f m driven)%n",
                    maneuver.name(), closure * 1000, headingError, result.totalDistanceMeters());
        } else {
            System.out.printf(
                    "[maneuver] %-26s along %+6.1f  cross %+6.1f  total %6.1f mm  "
                            + "heading %+6.2f deg%n",
                    maneuver.name(), along * 1000, cross * 1000,
                    result.totalErrorMeters() * 1000, headingError);
        }
    }

    /** Prints a ranked summary, worst first, so attention goes where it is needed. */
    public void printSummary() {
        if (results.isEmpty()) {
            System.out.println("[maneuver] no results recorded");
            return;
        }

        List<ManeuverResult> sorted = new ArrayList<>(results);
        sorted.sort((a, b) -> Double.compare(b.errorPerMeter(), a.errorPerMeter()));

        double tolerance = DriveAutoCalibrator.ACCEPTANCE_TOLERANCE_METERS;
        int passed = 0;
        for (ManeuverResult result : results) {
            if (result.passed(tolerance)) {
                passed++;
            }
        }

        System.out.println("=====================================================");
        System.out.printf(" MANOEUVRE SUMMARY — %d of %d within 1 inch%n", passed, results.size());
        System.out.println(" Worst error per metre driven, first:");
        for (int i = 0; i < Math.min(5, sorted.size()); i++) {
            ManeuverResult result = sorted.get(i);
            System.out.printf("   %-26s %5.2f mm/m  total %6.1f mm%n",
                    result.name(), result.errorPerMeter() * 1000,
                    result.totalErrorMeters() * 1000);
        }
        System.out.println("=====================================================");

        Logger.recordOutput("Calibration/Maneuvers/Summary/Count", results.size());
        Logger.recordOutput("Calibration/Maneuvers/Summary/Passed", passed);
        Logger.recordOutput("Calibration/Maneuvers/Summary/WorstName", sorted.get(0).name());
        Logger.recordOutput("Calibration/Maneuvers/Summary/WorstErrorPerMeterMm",
                sorted.get(0).errorPerMeter() * 1000);
    }

    /** @return the results recorded so far. */
    public List<ManeuverResult> getResults() {
        return List.copyOf(results);
    }

    /** Clears recorded results. */
    public void reset() {
        results.clear();
    }
}
