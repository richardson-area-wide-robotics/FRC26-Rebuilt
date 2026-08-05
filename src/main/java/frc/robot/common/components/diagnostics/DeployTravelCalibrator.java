package frc.robot.common.components.diagnostics;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.common.components.diagnostics.HardStopDetector.End;
import frc.robot.rebuilt.RebuiltConstants.IntakeConstants;
import frc.robot.rebuilt.RebuiltConstants.MechanismRatios;
import frc.robot.rebuilt.subsystems.Intake;

/**
 * Measures the intake arm's real travel by driving it gently onto each hard stop.
 *
 * <p>Closes a gap that has been open since the first review: {@code DEPLOY_POSITION_ROTATIONS = 10}
 * and soft limits of 0 to 11 are hand-chosen numbers in motor rotations, and nothing has ever checked
 * them against the arm. CAD could answer it, but so can the arm — and the arm has the advantage of
 * describing itself as built rather than as drawn.
 *
 * <p><b>Run it with no game pieces in the robot.</b> A ball under the arm would be found instead of
 * the stop. {@link HardStopDetector} would report it as an obstruction rather than mistaking it for a
 * stop, so the run fails honestly rather than producing a wrong number — but it is still a wasted run.
 *
 * <h2>What the numbers mean afterwards</h2>
 *
 * <ul>
 *   <li><b>Measured travel</b> is what the arm can actually do. If it is materially less than 10
 *       rotations, the deploy target is asking for travel the arm does not have, and the arm spends
 *       every deploy pressed against its stop drawing current.
 *   <li><b>Soft limits</b> should sit just inside the stops. Outside them they protect nothing;
 *       far inside them and they steal travel.
 *   <li><b>Encoder drift</b> at the stowed stop is the error in the constructor's assumption that the
 *       arm starts stowed. Anything non-zero means the soft limits were offset by the same amount.
 * </ul>
 */
public class DeployTravelCalibrator {

    // Seeks using the intake's own manual speeds rather than a special calibration speed. That is
    // deliberate: those are the speeds the arm actually meets its stops at in use, so the stop
    // positions measured here are the ones the running robot will see. A gentler calibration speed
    // would find slightly different positions, since how far the arm deflects into its stop depends
    // on how fast it arrives.

    /**
     * Seconds allowed to reach a stop before giving up.
     *
     * <p>A timeout rather than a hard requirement: if the arm has not arrived in this long, something
     * is wrong and holding output into it is not the answer.
     */
    private static final double SEEK_TIMEOUT = 4.0;

    /** Seconds to rest between the two directions. */
    private static final double REST_SECONDS = 1.0;

    private final Intake intake;

    /**
     * @param intake The intake, or null to skip.
     */
    public DeployTravelCalibrator(Intake intake) {
        this.intake = intake;
    }

    /**
     * Seeks one stop and stops as soon as it is confirmed.
     *
     * <p>Terminates on the detector rather than on the clock, so the arm is held against its stop only
     * as long as it takes to confirm — a stalled brushless motor turns nearly all its input into
     * winding heat.
     *
     * @param deployDirection true to seek the deployed stop, false for the stowed stop.
     * @return the command.
     */
    private Command seekStop(boolean deployDirection) {
        String label = deployDirection ? "deployed" : "stowed";

        return Commands.sequence(
                        Commands.runOnce(() -> System.out.println(
                                "[deploy] seeking the " + label + " stop")),
                        Commands.run(() -> {
                            if (deployDirection) {
                                intake.manualDeploy();
                            } else {
                                intake.manualReverseDeploy();
                            }
                        }, intake)
                                .until(() -> intake.getDeployStops().isAtHardStop())
                                .withTimeout(SEEK_TIMEOUT),
                        Commands.runOnce(() -> {
                            intake.stopDeploy();

                            HardStopDetector stops = intake.getDeployStops();
                            if (stops.isAtHardStop()) {
                                System.out.printf("[deploy] %s stop found at %.3f rotations%n",
                                        label, intake.getDeployPosition());
                            } else if (stops.isObstructed()) {
                                System.out.printf("[deploy] %s NOT found — something is in the way "
                                        + "and still moving. Clear the game pieces and re-run.%n",
                                        label);
                            } else if (stops.isAtSoftLimit()) {
                                System.out.printf("[deploy] %s NOT found — the SOFT LIMIT stopped the "
                                        + "arm first, at %.3f rotations. The limit is inside the "
                                        + "physical stop, so real travel is unknown from this run.%n",
                                        label, intake.getDeployPosition());
                            } else {
                                System.out.printf("[deploy] %s NOT found — timed out after %.1f s.%n",
                                        label, SEEK_TIMEOUT);
                            }
                        }, intake),
                        Commands.waitSeconds(REST_SECONDS))
                .withName("DeployTravel/" + label);
    }

    /**
     * Both stops, then the report.
     *
     * <p>Stowed first. It is the direction the arm idles in anyway, so it is the safer of the two to
     * discover a problem in.
     *
     * @return the command.
     */
    public Command full() {
        if (intake == null) {
            return Commands.runOnce(() ->
                    System.out.println("[deploy] skipped, no intake subsystem"));
        }

        return Commands.sequence(
                        Commands.runOnce(() -> {
                            intake.getDeployStops().resetLearned();
                            System.out.println("[deploy] === Intake arm travel ===");
                            System.out.println("[deploy] NO GAME PIECES IN THE ROBOT. A ball under "
                                    + "the arm gets found instead of the stop.");
                        }),
                        seekStop(false),
                        seekStop(true),
                        seekStop(false),
                        Commands.runOnce(this::printReport))
                .withName("DeployTravel/Full");
    }

    /** Prints the measured travel and what it says about the configured numbers. */
    public void printReport() {
        if (intake == null) {
            return;
        }

        HardStopDetector stops = intake.getDeployStops();
        double stowed = stops.getLearnedStop(End.LOW);
        double deployed = stops.getLearnedStop(End.HIGH);
        double travel = stops.getMeasuredTravel();

        System.out.println();
        System.out.println("=== INTAKE ARM TRAVEL REPORT ===");

        if (Double.isNaN(travel)) {
            System.out.println("  INCOMPLETE — one or both stops were not found. Nothing measured.");
            System.out.printf("  stowed stop:   %s%n", Double.isNaN(stowed) ? "not found" : stowed);
            System.out.printf("  deployed stop: %s%n",
                    Double.isNaN(deployed) ? "not found" : deployed);
            System.out.println("=== END ===");
            return;
        }

        System.out.printf("  stowed stop    %.3f rotations  (%d hits, spread %.3f)%n",
                stowed, stops.getStopHits(End.LOW), stops.getStopSpread(End.LOW));
        System.out.printf("  deployed stop  %.3f rotations  (%d hits, spread %.3f)%n",
                deployed, stops.getStopHits(End.HIGH), stops.getStopSpread(End.HIGH));
        System.out.printf("  MEASURED TRAVEL %.3f rotations", travel);

        if (MechanismRatios.INTAKE_DEPLOY_REDUCTION != 1.0) {
            System.out.printf("  =  %.1f degrees of arm%n",
                    MechanismRatios.deployRotationsToDegrees(travel));
        } else {
            System.out.printf("%n  (set MechanismRatios.INTAKE_DEPLOY_REDUCTION to see this in "
                    + "arm degrees)%n");
        }

        System.out.println();

        // The repeatability check. Two visits to the stowed stop should agree; if they do not, the
        // encoder is losing count or the stop itself is moving.
        double spread = stops.getStopSpread(End.LOW);
        if (stops.getStopHits(End.LOW) >= 2 && spread > 0.2) {
            System.out.printf("  WARNING: the stowed stop moved %.3f rotations between visits.%n",
                    spread);
            System.out.println("  Either the encoder is losing count or the stop is not rigid —");
            System.out.println("  a fastener backing out would look exactly like this. Neither is");
            System.out.println("  a calibration problem, so fix it before trusting any of the above.");
            System.out.println();
        }

        // Against the configured deploy target.
        double target = IntakeConstants.DEPLOY_POSITION_ROTATIONS;
        System.out.printf("  configured deploy target      %.1f rotations%n", target);
        if (target > travel) {
            System.out.printf("  ^ ASKS FOR MORE TRAVEL THAN EXISTS by %.2f rotations. The arm will "
                    + "sit against%n    its stop drawing current on every deploy.%n", target - travel);
        } else if (travel - target > 1.0) {
            System.out.printf("  ^ leaves %.2f rotations of travel unused%n", travel - target);
        } else {
            System.out.println("  ^ sits sensibly inside the travel");
        }

        // Against the soft limits, which is the check that has been open since the first review.
        System.out.printf("%n  soft limits  %.1f to %.1f rotations%n",
                IntakeConstants.DEPLOY_REVERSE_SOFT_LIMIT,
                IntakeConstants.DEPLOY_FORWARD_SOFT_LIMIT);
        if (IntakeConstants.DEPLOY_FORWARD_SOFT_LIMIT > deployed) {
            System.out.println("  ^ the forward limit is OUTSIDE the physical stop, so it protects");
            System.out.println("    nothing — the arm reaches steel before the limit engages.");
        }
        if (IntakeConstants.DEPLOY_REVERSE_SOFT_LIMIT < stowed) {
            System.out.println("  ^ the reverse limit is OUTSIDE the stowed stop, same problem.");
        }

        System.out.println();
        System.out.println("  Remember the soft limits are in the SAME UNITS as the encoder, so they");
        System.out.println("  inherit any error in its zero. The constructor zeroes at boot assuming");
        System.out.println("  the arm is stowed; if it ever boots part-way, the limits are offset by");
        System.out.println("  exactly that much. Intake.rezeroDeployAtStowedStop() corrects it");
        System.out.println("  against the physical stop instead of against the assumption.");
        System.out.println("=== END ===");
        System.out.println();
    }
}
