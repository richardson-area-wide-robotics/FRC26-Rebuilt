package frc.robot.common.components.diagnostics;

import frc.robot.common.components.diagnostics.VisionCalibration.RunningStats;
import org.littletonrobotics.junction.Logger;

/**
 * Measures the current thresholds that game piece and jam detection depend on.
 *
 * <p>{@link MotorLoadMonitor} needs three numbers per mechanism: how many amps above idle indicate a
 * piece is moving through, what speed the mechanism runs at unloaded, and how far the speed must
 * fall before it counts as stuck. Guessing them produces a detector that either misses pieces or
 * decides it is jammed while doing its job. All three are directly measurable.
 *
 * <p>The routine has two phases. Run the mechanism <b>empty</b> to learn the idle current, its noise,
 * and the free-running speed. Then send <b>game pieces</b> through and learn the loaded current and
 * the speed a piece drags it down to. The thresholds fall out of the gap between them.
 *
 * <h2>Which loaded samples actually had a piece in them</h2>
 *
 * <p>The awkward part: during the loaded phase, pieces are going through only some of the time, and
 * there is no sensor to say when — that sensor is the thing being calibrated. Averaging the whole
 * phase mixes loaded and unloaded samples together and understates the loaded current by however
 * much of the phase was idle.
 *
 * <p>So samples above the empty phase's noise floor are attributed to a piece, and the rest are not.
 * That is a bootstrap, not circular reasoning: the floor comes entirely from the empty phase, which
 * was measured with nothing going through and knows nothing about the loaded phase.
 *
 * <p>It does mean the selected population is guaranteed to sit above the floor, so its separation
 * from idle proves nothing. <b>Viability is therefore judged on the unselected phase mean</b>, which
 * no selection has touched. A mechanism where pieces barely register shows a low separation there no
 * matter how cleanly the few crossing samples are picked out.
 *
 * <p><b>The most useful thing this can tell you is that it will not work.</b> If loaded current is
 * not clearly separated from the noise on the idle current, no threshold exists that reliably
 * distinguishes them, and {@link Recommendation#isViable()} says so instead of emitting a number
 * that would misfire all match. That is a real possibility for a lightly-loaded roller, and finding
 * out in the shop is far better than finding out from a piece count that drifts during a match.
 */
public class LoadCalibrator {

    /**
     * How many standard deviations above idle noise a threshold must sit.
     *
     * <p>3 sigma means roughly one false trigger in 370 samples from noise alone — about once every
     * seven seconds at a 20 ms loop, which sounds bad until the sustain requirement in
     * {@link GamePieceCounter} is applied on top, needing several consecutive samples.
     */
    private static final double NOISE_MARGIN_SIGMA = 3.0;

    /**
     * Fraction of the empty-to-loaded gap the threshold sits at.
     *
     * <p>0.5 puts it midway. Lower catches more marginal pieces at the cost of false positives.
     */
    private static final double GAP_FRACTION = 0.5;

    /** Separation below this, in sigma, means the two populations overlap too much to tell apart. */
    private static final double MIN_VIABLE_SEPARATION_SIGMA = 4.0;

    /** Empty samples needed before the noise floor is trusted enough to segment against. */
    private static final int MIN_EMPTY_SAMPLES = 50;

    /** Selected loaded samples needed for the loaded population to mean anything. */
    private static final int MIN_LOADED_SAMPLES = 25;

    /** The measured thresholds, ready to paste into {@code LoadConstants}. */
    public record Recommendation(
            String mechanism,
            double idleAmps,
            double idleAmpsStdDev,
            double loadedAmps,
            double loadedPeakAmps,
            double workExcessAmps,
            double expectedSpeed,
            double loadedSpeedFraction,
            double jamSpeedFraction,
            double separationSigma,
            double pieceDutyCycle,
            double jamAmps,
            double observedJamSpeedFraction,
            int emptySamples,
            int loadedPhaseSamples,
            int loadedSamples,
            int jamSamples) {

        /**
         * @return true when the jam threshold sits between two measured populations rather than
         *     being inferred from the working one alone. An unvalidated jam threshold is usable but
         *     is a guess, and the report says which it is.
         */
        public boolean isJamThresholdMeasured() {
            return jamSamples >= MIN_LOADED_SAMPLES && !Double.isNaN(observedJamSpeedFraction);
        }

        /**
         * Whether the two current populations are far enough apart to distinguish.
         *
         * <p>Judged on {@link #separationSigma()}, which is computed from the whole loaded phase
         * rather than the samples selected out of it — see the class docs for why that distinction
         * decides whether this method means anything.
         *
         * @return true when a threshold can reliably separate loaded from empty.
         */
        public boolean isViable() {
            return emptySamples >= MIN_EMPTY_SAMPLES
                    && loadedSamples >= MIN_LOADED_SAMPLES
                    && separationSigma >= MIN_VIABLE_SEPARATION_SIGMA
                    && workExcessAmps > 0;
        }

        /** @return a paste-ready constants line, or an explanation of why there is none. */
        public String describe() {
            if (emptySamples < MIN_EMPTY_SAMPLES) {
                return String.format(
                        "%s: INCOMPLETE — only %d empty samples, need %d. Run the empty phase longer.",
                        mechanism, emptySamples, MIN_EMPTY_SAMPLES);
            }

            if (loadedSamples < MIN_LOADED_SAMPLES) {
                return String.format(
                        "%s: INCOMPLETE — only %d of %d loaded-phase samples rose above the noise "
                                + "floor (need %d). Either too few pieces went through, or a piece "
                                + "does not load this mechanism enough to see. Idle %.1f +/- %.2f A.",
                        mechanism, loadedSamples, loadedPhaseSamples, MIN_LOADED_SAMPLES,
                        idleAmps, idleAmpsStdDev);
            }

            if (!isViable()) {
                return String.format(
                        "%s: NOT VIABLE — across the whole loaded phase the mean was only %.1f "
                                + "sigma above idle %.1f A (noise %.2f A), below the %.1f sigma "
                                + "needed. Current-based detection on this mechanism would misfire. "
                                + "Pieces were present for about %.0f%% of the phase.",
                        mechanism, separationSigma, idleAmps, idleAmpsStdDev,
                        MIN_VIABLE_SEPARATION_SIGMA, pieceDutyCycle * 100);
            }

            String jamNote = isJamThresholdMeasured()
                    ? String.format(
                            "obstructed at %.0f%% of free speed drawing %.1f A, so the threshold "
                                    + "sits between two measured populations",
                            observedJamSpeedFraction * 100, jamAmps)
                    : "jam threshold INFERRED, not measured — no obstructed phase was run";

            return String.format(
                    "%s: WORK_EXCESS_AMPS = %.1f;  EXPECTED_RPM = %.0f;  jam fraction %.2f  "
                            + "(idle %.1f +/- %.2f A, loaded %.1f A, peak %.1f A, %.1f sigma apart; "
                            + "a piece drags speed to %.0f%%; pieces present %.0f%% of the phase; %s)",
                    mechanism, workExcessAmps, expectedSpeed, jamSpeedFraction,
                    idleAmps, idleAmpsStdDev, loadedAmps, loadedPeakAmps, separationSigma,
                    loadedSpeedFraction * 100, pieceDutyCycle * 100, jamNote);
        }
    }

    private final String mechanism;

    private final RunningStats emptyAmps = new RunningStats();
    private final RunningStats emptySpeed = new RunningStats();

    /** Every sample from the loaded phase, selected or not. Viability is judged on this. */
    private final RunningStats loadedPhaseAmps = new RunningStats();

    /** Only the loaded-phase samples attributed to a piece being present. */
    private final RunningStats loadedAmps = new RunningStats();
    private final RunningStats loadedSpeed = new RunningStats();

    /** Samples taken with the mechanism deliberately obstructed. Optional. */
    private final RunningStats jamAmps = new RunningStats();
    private final RunningStats jamSpeed = new RunningStats();

    /**
     * Discards every population, so a re-run replaces the previous attempt rather than joining it.
     *
     * <p>Needed the moment a routine can be re-run on the operator's say-so, and its absence was
     * quiet: re-running merged both attempts into one population. A first attempt with a game piece
     * accidentally left in the empty phase would then inflate the noise floor for ever -- and the noise
     * floor is what every later sample is segmented against, so one bad run silently degrades every
     * good one after it. Doubled sample counts were the only visible tell.
     */
    public void reset() {
        emptyAmps.reset();
        emptySpeed.reset();
        loadedPhaseAmps.reset();
        loadedAmps.reset();
        loadedSpeed.reset();
        jamAmps.reset();
        jamSpeed.reset();
    }

    /**
     * @param mechanism Name for the report, e.g. "INTAKE".
     */
    public LoadCalibrator(String mechanism) {
        this.mechanism = mechanism;
    }

    /**
     * Records a sample with nothing going through the mechanism.
     *
     * @param amps  Motor current.
     * @param speed Mechanism speed, any consistent unit.
     */
    public void addEmptySample(double amps, double speed) {
        emptyAmps.add(amps);
        emptySpeed.add(Math.abs(speed));
    }

    /**
     * Records a sample from the loaded phase, whether or not a piece happens to be present.
     *
     * <p>Feed every sample from the phase, including the gaps between pieces. Segmentation happens
     * here against the empty phase's noise floor — filtering beforehand would remove the unselected
     * population that {@link Recommendation#isViable()} depends on.
     *
     * @param amps  Motor current.
     * @param speed Mechanism speed, same unit as the empty phase.
     * @return true if this sample was attributed to a game piece.
     */
    public boolean addLoadedSample(double amps, double speed) {
        loadedPhaseAmps.add(amps);

        if (amps >= pieceFloorAmps()) {
            loadedAmps.add(amps);
            loadedSpeed.add(Math.abs(speed));
            return true;
        }

        return false;
    }

    /**
     * Records a sample with the mechanism deliberately obstructed.
     *
     * <p>Optional but worth doing: without it the jam speed threshold is only ever inferred from how
     * far a <em>working</em> mechanism slows, which says nothing about how far a stuck one does. With
     * it the threshold can be placed between two measured populations instead of below one.
     *
     * <p>Obstruct by hand with something that will not damage the mechanism, and keep the phase
     * short. A brushless motor held near stall dumps its power into heat.
     *
     * @param amps  Motor current.
     * @param speed Mechanism speed, same unit as the other phases.
     */
    public void addJamSample(double amps, double speed) {
        jamAmps.add(amps);
        jamSpeed.add(Math.abs(speed));
    }

    /**
     * @return the current above which a loaded-phase sample is taken to have a piece in it.
     *
     *     <p>Before the empty phase has enough samples to have a meaningful noise figure there is
     *     nothing to segment against, so everything counts. That case is caught separately by the
     *     sample-count check in {@link Recommendation#isViable()} rather than silently producing a
     *     number from an unlearned floor.
     */
    private double pieceFloorAmps() {
        if (emptyAmps.getCount() < MIN_EMPTY_SAMPLES) {
            return Double.NEGATIVE_INFINITY;
        }
        return emptyAmps.getMean() + NOISE_MARGIN_SIGMA * emptyAmps.getStdDev();
    }

    /**
     * Works the thresholds out from the two phases.
     *
     * @return the recommendation, viable or not.
     */
    public Recommendation recommend() {
        double idle = emptyAmps.getMean();
        double noise = emptyAmps.getStdDev();
        double loaded = loadedAmps.getMean();
        double freeSpeed = emptySpeed.getMean();

        // Separation, and it took two attempts to get right. Both failure modes are worth keeping,
        // because they are opposite and each looks reasonable on its own.
        //
        // It was originally the UNSELECTED phase mean against idle, to avoid the obvious circularity:
        // the selected samples were chosen for clearing the floor, so of course they clear it. But the
        // unselected mean is DILUTED by the gaps between pieces. With duty cycle D and true excess E
        // over sigma, that figure is D*E/sigma -- so a 4-sigma bar silently demanded a true separation
        // of 4/D, and at the 20% duty this routine's own docs anticipate that is a 20-sigma bar. A
        // genuinely excellent 14-sigma mechanism computed as 2.8 and was condemned as NOT VIABLE, which
        // sends somebody looking for a beam-break sensor a working intake did not need.
        //
        // Dividing by the observed duty cycle looks like the fix and is worse. The "observed duty" is
        // the SELECTED fraction, and when the populations genuinely overlap that fraction collapses to
        // the noise tail -- so dividing by it manufactures separation exactly where there is none. It
        // turns the one case that must fail into a pass.
        //
        // What works is to ask the right question of the selected population: is its mean higher than
        // truncation ALONE would produce? Selecting the top tail of pure noise yields a mean of about
        // (k + 1/k) sigma above idle for a k-sigma cut. Anything materially above that is signal, and
        // nothing about it is diluted by duty cycle or circular.
        double noiseOnlyMean = idle
                + (NOISE_MARGIN_SIGMA + 1.0 / NOISE_MARGIN_SIGMA) * noise;
        double separationSigma =
                noise > 1e-6 ? (loadedAmps.getMean() - noiseOnlyMean) / noise : 0;

        double observedDuty = loadedPhaseAmps.getCount() == 0
                ? 0
                : (double) loadedAmps.getCount() / loadedPhaseAmps.getCount();

        // The threshold has to clear the noise AND sit meaningfully below the loaded level. Take
        // whichever constraint binds harder rather than assuming one always does.
        double noiseFloor = NOISE_MARGIN_SIGMA * noise;
        double halfGap = GAP_FRACTION * (loaded - idle);
        double workExcess = Math.max(noiseFloor, halfGap);

        // A piece drags the mechanism down to some fraction of free speed. The jam threshold must
        // sit below that, or every piece going through reads as a jam.
        double loadedFraction = freeSpeed > 1e-6 ? loadedSpeed.getMean() / freeSpeed : 0;
        double jamFraction;
        double jamFractionObserved = freeSpeed > 1e-6 && jamSpeed.getCount() > 0
                ? jamSpeed.getMean() / freeSpeed
                : Double.NaN;

        if (Double.isNaN(jamFractionObserved)) {
            // No obstructed phase was run, so all that is known is how far a working mechanism
            // slows. Sit well below that and accept it is a guess, not a measurement.
            jamFraction = Math.max(0.10, Math.min(0.50, loadedFraction * 0.5));
        } else {
            // Both populations measured: put the threshold between them. Anything above is the
            // mechanism working, anything below is it stuck, and the margin either side is real.
            jamFraction = Math.max(0.05,
                    Math.min(0.60, (loadedFraction + jamFractionObserved) / 2.0));
        }

        double dutyCycle = observedDuty;

        return new Recommendation(
                mechanism, idle, noise, loaded, loadedAmps.getMax(), workExcess, freeSpeed,
                loadedFraction, jamFraction, separationSigma, dutyCycle,
                jamAmps.getMean(), jamFractionObserved,
                emptyAmps.getCount(), loadedPhaseAmps.getCount(), loadedAmps.getCount(),
                jamAmps.getCount());
    }

    /** Publishes progress, so an operator can see the phases filling up. */
    public void log() {
        String root = "LoadCalibration/" + mechanism;
        Logger.recordOutput(root + "/EmptySamples", emptyAmps.getCount());
        Logger.recordOutput(root + "/LoadedPhaseSamples", loadedPhaseAmps.getCount());
        Logger.recordOutput(root + "/LoadedSamples", loadedAmps.getCount());
        Logger.recordOutput(root + "/IdleAmps", emptyAmps.getMean());
        Logger.recordOutput(root + "/IdleAmpsStdDev", emptyAmps.getStdDev());
        Logger.recordOutput(root + "/PieceFloorAmps", pieceFloorAmps());
        Logger.recordOutput(root + "/LoadedAmps", loadedAmps.getMean());
        Logger.recordOutput(root + "/FreeSpeed", emptySpeed.getMean());
        Logger.recordOutput(root + "/LoadedSpeed", loadedSpeed.getMean());

        Recommendation recommendation = recommend();
        Logger.recordOutput(root + "/SeparationSigma", recommendation.separationSigma());
        Logger.recordOutput(root + "/PieceDutyCycle", recommendation.pieceDutyCycle());
        Logger.recordOutput(root + "/JamSamples", jamAmps.getCount());
        Logger.recordOutput(root + "/JamAmps", jamAmps.getMean());
        Logger.recordOutput(root + "/JamSpeed", jamSpeed.getMean());
        Logger.recordOutput(root + "/Viable", recommendation.isViable());
        Logger.recordOutput(root + "/JamThresholdMeasured", recommendation.isJamThresholdMeasured());
        Logger.recordOutput(root + "/RecommendedWorkExcessAmps", recommendation.workExcessAmps());
        Logger.recordOutput(root + "/RecommendedJamFraction", recommendation.jamSpeedFraction());
    }

    /** @return samples recorded in the empty phase. */
    public int getEmptySamples() {
        return emptyAmps.getCount();
    }

    /** @return samples recorded in the loaded phase, including those without a piece present. */
    public int getLoadedPhaseSamples() {
        return loadedPhaseAmps.getCount();
    }

    /** @return loaded-phase samples attributed to a game piece. */
    public int getLoadedSamples() {
        return loadedAmps.getCount();
    }

    /** @return the mechanism this calibrator is measuring. */
    public String getMechanism() {
        return mechanism;
    }
}
