package frc.robot.common.components.diagnostics;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.components.diagnostics.GatedStep.Assessment;
import frc.robot.common.components.diagnostics.HardStopDetector.End;
import frc.robot.rebuilt.subsystems.Intake;
import java.util.ArrayList;
import java.util.List;

/**
 * Adapters that let the existing calibrators run under {@link GuidedCalibration}.
 *
 * <p>Thin on purpose. Each one wires an existing routine's command to its existing result object and
 * translates that result into a verdict &mdash; no measurement logic lives here. What is new is only
 * the translation, and that is where the value is: it turns a printed report a human had to interpret
 * into a decision the sequence can act on.
 *
 * <p><b>Every {@code retry} message names what to change.</b> A verdict that only says the data was
 * bad invites the identical run a second time, and the identical verdict a second time.
 */
public final class CalibrationSteps {

    private CalibrationSteps() {
    }

    /**
     * The drive feedforward, from a real SysId sweep.
     *
     * @param sysId The characterisation routine.
     * @return the gated step.
     */
    public static GatedStep driveFeedforward(DriveSysId sysId) {
        return new GatedStep() {
            @Override
            public String name() {
                return "DRIVE FEEDFORWARD (SysId)";
            }

            @Override
            public String setupPrompt() {
                return "Robot at ONE END of the carpet, at least 28 ft of clear run ahead, bumpers "
                        + "on, good battery. It will drive forward and back four times.";
            }

            @Override
            public Command measure() {
                return sysId.full();
            }

            @Override
            public void reset() {
                sysId.reset();
            }

            @Override
            public Assessment assess() {
                DriveSysId.Summary summary = sysId.summarise();

                if (summary.trustworthyModules() == 0) {
                    return Assessment.retry("No module produced a usable fit. Almost always means "
                            + "the run was too short to build up speed, or the robot never moved. "
                            + "Check it has the full run and re-run.");
                }
                if (!summary.isComplete()) {
                    return Assessment.retry(String.format(
                            "Only %d of 4 modules fitted. A mean built from the rest would be low, "
                            + "so it is being withheld. Check the drive encoder on the module(s) "
                            + "missing from the per-module list above, then re-run.",
                            summary.trustworthyModules()));
                }
                if (summary.kaSpreadPercent() > 25.0) {
                    return Assessment.retry(String.format(
                            "kA spread %.0f%% across modules, highest at %s. One corner accelerates "
                            + "differently, which a chassis feedforward cannot represent. Check that "
                            + "corner mechanically -- if it is genuinely fine, re-run to see whether "
                            + "the spread repeats.",
                            summary.kaSpreadPercent(), summary.worstModule()));
                }
                if (summary.kV() <= 0) {
                    return Assessment.fail("kV came out non-positive, which is not physical. That is "
                            + "a sign error somewhere upstream, not a bad run -- do the hand-motion "
                            + "check in step 1b before running this again.");
                }

                return Assessment.pass(String.format(
                        "kS = %.4f, kV = %.4f, kA = %.4f from all 4 modules, spread %.0f%%. "
                        + "Run tools/apply_sysid.py to write these in.",
                        summary.kS(), summary.kV(), summary.kA(), summary.kaSpreadPercent()));
            }
        };
    }

    /**
     * The drive current limit, from pushing a wall until the wheels break loose.
     *
     * @param traction The traction sweep.
     * @return the gated step.
     */
    public static GatedStep tractionLimit(TractionCalibrator traction) {
        return new GatedStep() {
            @Override
            public String name() {
                return "TRACTION LIMIT";
            }

            @Override
            public String setupPrompt() {
                return "Robot squared against a solid wall ON CARPET, bumpers flat against it across "
                        + "their full width, good battery. It will push harder each step.";
            }

            @Override
            public Command measure() {
                return traction.sweep();
            }

            @Override
            public Assessment assess() {
                TractionCalibrator.Result result = traction.analyse();

                if (result.aborted()) {
                    return Assessment.retry(result.abortReason());
                }
                if (!result.foundTractionLimit()) {
                    return Assessment.pass("No slip up to the highest limit tried, so traction is not "
                            + "what bounds this drivetrain. Nothing to change -- the cap stands.");
                }

                String breaker = result.breakerWarning();
                return Assessment.pass(String.format(
                        "Slip at %d A per motor, so the recommendation is %d A.%s",
                        result.tractionLimitAmps(), result.recommendedAmps(),
                        breaker.isEmpty() ? "" : " " + breaker));
            }
        };
    }

    /**
     * The intake arm's motion profile: break-away, kV, gravity and achievable limits.
     *
     * @param profile The arm profile calibration.
     * @return the gated step.
     */
    public static GatedStep armProfile(ArmProfileCalibrator profile) {
        return new GatedStep() {
            @Override
            public String name() {
                return "ARM MOTION PROFILE";
            }

            @Override
            public String setupPrompt() {
                return "Robot on blocks, NO game pieces, nothing under the arm. Run ARM TRAVEL first "
                        + "-- the gravity phase drives to fractions of the measured travel and is "
                        + "skipped without it.";
            }

            @Override
            public Command measure() {
                return profile.full();
            }

            @Override
            public void reset() {
                profile.reset();
            }

            @Override
            public Assessment assess() {
                ArmProfileCalibrator.Outcome outcome = profile.outcome();

                if (!outcome.breakawayMeasuredBothWays()) {
                    return Assessment.retry("A break-away ramp never saw the arm move, so friction and "
                            + "gravity cannot be separated. Almost always a soft limit cutting output "
                            + "at one end -- check them and re-run.");
                }
                if (!outcome.kvFit()) {
                    return Assessment.retry("The velocity fit is not trustworthy. Steps were discarded "
                            + "for hitting a stop or stalling, so there is not enough clean travel. "
                            + "Check the arm swings freely across its whole range, then re-run.");
                }
                if (outcome.holdSamples() == 0) {
                    return Assessment.retry("No holding-voltage sample survived. Either travel was "
                            + "never measured, or the arm did not reach the positions it was asked for "
                            + "-- which would record gravity against the wrong angle. Run ARM TRAVEL "
                            + "first, then re-run.");
                }

                return Assessment.pass(String.format(
                        "Break-away both ways, kV fitted, %d/5 hold samples, peak %.1f rot/s and "
                        + "%.0f rot/s2. Compare those against the configured profile limits.",
                        outcome.holdSamples(), outcome.peakVelocityRps(), outcome.peakAccelRps2()));
            }
        };
    }

    /**
     * Current thresholds for game-piece and jam detection, across all four mechanisms.
     *
     * @param routine The load calibration.
     * @return the gated step.
     */
    public static GatedStep loadThresholds(LoadCalibrationRoutine routine) {
        return new GatedStep() {
            @Override
            public String name() {
                return "LOAD THRESHOLDS";
            }

            @Override
            public String setupPrompt() {
                return "Robot on blocks. About 20 game pieces and a helper. Nothing loaded when it "
                        + "starts -- the empty phase runs first, and a piece left in it inflates the "
                        + "noise floor everything else is judged against.";
            }

            @Override
            public Command measure() {
                return routine.full();
            }

            @Override
            public void reset() {
                routine.reset();
            }

            @Override
            public Assessment assess() {
                List<LoadCalibrator.Recommendation> all = routine.recommendations();

                List<String> notViable = new ArrayList<>();
                for (LoadCalibrator.Recommendation recommendation : all) {
                    if (!recommendation.isViable()) {
                        notViable.add(recommendation.mechanism());
                    }
                }

                if (notViable.size() == all.size()) {
                    return Assessment.retry("No mechanism separated loaded from empty. That is more "
                            + "likely a phase that did not happen than four bad mechanisms -- feed "
                            + "pieces continuously through the whole LOADED phase, and obstruct the "
                            + "mechanism properly during the OBSTRUCTED phase. Then re-run.");
                }
                if (!notViable.isEmpty()) {
                    return Assessment.retry("Did not separate on: " + String.join(", ", notViable)
                            + ". Feed pieces through those more continuously and re-run. If it repeats, "
                            + "current sensing genuinely will not work there and it needs a sensor.");
                }

                return Assessment.pass("All " + all.size() + " mechanisms separated loaded from empty. "
                        + "Read the per-mechanism numbers before pasting -- they depend on which game "
                        + "piece was used and how worn the rollers are.");
            }
        };
    }

    /**
     * Rotational inertia, in both intake states.
     *
     * @param inertia The inertia calibration.
     * @return the gated step.
     */
    public static GatedStep rotationalInertia(RotationalInertiaCalibrator inertia) {
        return new GatedStep() {
            @Override
            public String name() {
                return "ROTATIONAL INERTIA";
            }

            @Override
            public String setupPrompt() {
                return "Robot on the floor with about 2 m clear all round. It will spin in place twice "
                        + "-- once stowed, once deployed -- because deploying moves several kilograms "
                        + "outward and inertia goes as mass times radius squared.";
            }

            @Override
            public Command measure() {
                return inertia.full();
            }

            @Override
            public Assessment assess() {
                RotationalInertiaCalibrator.Result stowed = inertia.getStowedResult();
                RotationalInertiaCalibrator.Result deployed = inertia.getDeployedResult();

                if (stowed == null || deployed == null) {
                    return Assessment.retry("One of the two states never produced a run. Check the arm "
                            + "actually deployed and stowed between them, then re-run.");
                }
                if (!stowed.isValid() || !deployed.isValid()) {
                    String which = !stowed.isValid() ? "stowed" : "deployed";
                    return Assessment.retry("The " + which + " run is not believable -- too few "
                            + "samples, a poor straight-line fit, or the wheels outran the spin rate. "
                            + "Make sure it has room to spin freely on carpet and re-run. Do NOT lower "
                            + "the spin voltage: that lowers the spin rate and makes the check worse.");
                }

                return Assessment.pass(String.format(
                        "I = %.3f stowed, %.3f deployed kg.m2. Treat these as upper bounds -- they "
                        + "come from motor current through a datasheet torque constant, with no "
                        + "allowance for gearbox efficiency.",
                        stowed.momentOfInertia(), deployed.momentOfInertia()));
            }
        };
    }

    /**
     * The intake arm's travel, driven against its own hard stops.
     *
     * @param travel The powered travel calibration.
     * @param intake The arm, for reading the learned stops.
     * @return the gated step.
     */
    public static GatedStep armTravel(DeployTravelCalibrator travel, Intake intake) {
        return new GatedStep() {
            @Override
            public String name() {
                return "ARM TRAVEL (powered)";
            }

            @Override
            public String setupPrompt() {
                return "Robot on blocks, NO game pieces near the intake, nothing under the arm. It "
                        + "will drive the arm to each stop. Step 1b should already have measured the "
                        + "same span by hand -- this must agree with it.";
            }

            @Override
            public Command measure() {
                return travel.full();
            }

            @Override
            public void reset() {
                // Learned stops are what this step measures, so a retry has to forget the previous
                // attempt's. Otherwise a run that mislearned a stop keeps that stop for ever, and the
                // goal clamp silently inherits it.
                intake.getDeployStops().resetLearned();
            }

            @Override
            public Assessment assess() {
                HardStopDetector stops = intake.getDeployStops();
                double measured = stops.getMeasuredTravel();

                if (Double.isNaN(measured)) {
                    boolean lowFound = !Double.isNaN(stops.getLearnedStop(End.LOW));
                    boolean highFound = !Double.isNaN(stops.getLearnedStop(End.HIGH));
                    String which = lowFound ? "the DEPLOYED stop" : highFound ? "the STOWED stop"
                            : "NEITHER stop";

                    return Assessment.retry(which + " was never found. Either a soft limit is cutting "
                            + "output before the arm reaches steel, or a game piece is under the arm "
                            + "and it stalled early. Clear the arm and check the soft limits, then "
                            + "re-run.");
                }

                return Assessment.pass(String.format(
                        "Travel %.2f rotations, stops at %.2f and %.2f. Check this against the "
                        + "by-hand figure from step 1b before pasting anything.",
                        Math.abs(measured), stops.getLearnedStop(End.LOW),
                        stops.getLearnedStop(End.HIGH)));
            }
        };
    }
}
