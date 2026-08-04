package frc.robot.common.components.diagnostics;

import org.littletonrobotics.junction.Logger;

/**
 * Counts game pieces past a mechanism from its current signature.
 *
 * <p>The robot has no game-piece sensor anywhere — {@code EasyBreakBeam} exists in the framework and
 * is never instantiated — so indexing is entirely driver-timed and nothing knows how many pieces are
 * aboard. A current spike is the sensor that is already fitted.
 *
 * <p>Counting rising edges alone does not work. Current wobbles around any threshold, so a single
 * piece produces several crossings and the count runs away. Two guards fix it:
 *
 * <ul>
 *   <li><b>Sustain</b> — the elevated reading must persist for several loops before it counts, which
 *       rejects the brief spikes from a roller biting on nothing.</li>
 *   <li><b>Refractory period</b> — after a count, further detections are ignored for a while. Set it
 *       from the fastest the mechanism can physically pass two pieces; anything sooner is the same
 *       piece being seen twice.</li>
 * </ul>
 *
 * <p>This is a count of <em>detections</em>, not ground truth. It will miss a piece that slides
 * through without loading the roller, and it will over-count if the refractory period is too short.
 * Treat it as a strong hint for the driver, not as an interlock — nothing safety-critical should
 * depend on it until it has been checked against a hopper counted by hand.
 */
public class GamePieceCounter {

    private final String name;
    private final MotorLoadMonitor monitor;

    /** Loops the elevated reading must persist before it counts as a piece. */
    private final int sustainLoops;

    /** Loops to ignore further detections after a count. */
    private final int refractoryLoops;

    private int count;
    private int sustainedLoops;
    private int refractoryRemaining;
    private boolean countedThisEvent;

    /**
     * @param name            Log prefix, e.g. "Intake".
     * @param monitor         Load monitor for the mechanism.
     * @param sustainLoops    Loops of sustained work before counting. 3 loops is 60 ms.
     * @param refractoryLoops Loops to ignore after counting. Set from the fastest real piece rate.
     */
    public GamePieceCounter(String name, MotorLoadMonitor monitor, int sustainLoops,
            int refractoryLoops) {
        this.name = name;
        this.monitor = monitor;
        this.sustainLoops = sustainLoops;
        this.refractoryLoops = refractoryLoops;
    }

    /**
     * Folds in the current load state. Call once per loop, after the monitor has been updated.
     *
     * @return true on the loop a new piece is counted, so callers can trigger a rumble or a light.
     */
    public boolean update() {
        boolean newPiece = false;

        if (refractoryRemaining > 0) {
            refractoryRemaining--;
        }

        if (monitor.isDoingWork()) {
            sustainedLoops++;

            if (sustainedLoops >= sustainLoops && !countedThisEvent && refractoryRemaining == 0) {
                count++;
                countedThisEvent = true;
                refractoryRemaining = refractoryLoops;
                newPiece = true;
                System.out.printf("[piece] %s detected piece #%d (%.1f A over baseline)%n",
                        name, count, monitor.getExcessCurrent());
            }
        } else {
            // The load has come off, so the next elevation is a genuinely new event.
            sustainedLoops = 0;
            countedThisEvent = false;
        }

        Logger.recordOutput("Pieces/" + name + "/Count", count);
        Logger.recordOutput("Pieces/" + name + "/Detecting", monitor.isDoingWork());
        Logger.recordOutput("Pieces/" + name + "/RefractoryRemaining", refractoryRemaining);

        return newPiece;
    }

    /** @return pieces detected since the last reset. */
    public int getCount() {
        return count;
    }

    /** Resets the count, e.g. at the start of a match. */
    public void reset() {
        count = 0;
        sustainedLoops = 0;
        refractoryRemaining = 0;
        countedThisEvent = false;
    }

    /** @return true while a piece is currently being detected. */
    public boolean isDetecting() {
        return monitor.isDoingWork();
    }

    /** @return this counter's log prefix. */
    public String getName() {
        return name;
    }
}
