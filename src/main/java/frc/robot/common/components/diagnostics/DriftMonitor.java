package frc.robot.common.components.diagnostics;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import org.littletonrobotics.junction.Logger;

/**
 * Watches live estimates against the values the robot is running on, and says when they have parted
 * company.
 *
 * <p>This is the mechanism for noticing that the robot has broken in. Tread wears, gears bed in,
 * belts stretch — so a wheel diameter that was right in week one drifts, and the usual symptom is
 * somebody re-tuning gains to compensate for a geometry error that has crept in underneath them.
 *
 * <p><b>It observes and reports. It never applies anything.</b> Promotion is a deliberate act with
 * a human present, for the reason set out in {@link CalibrationStore#mayAutoAdopt}: some quantities
 * are measured independently of themselves and some are measured against the very thing they
 * determine. Automatically adopting the second kind closes a feedback loop with nothing to stop it
 * walking away from truth.
 *
 * <p>Each watched value publishes under {@code Calibration/Drift/}: the value in use, the live
 * estimate, the difference as a percentage, whether there is yet enough evidence to believe it, and
 * whether it has exceeded its threshold. A single boolean, {@code AnyDrifted}, is the one to put on
 * a dashboard.
 */
public class DriftMonitor {

    /** One quantity being watched. */
    public static final class Watch {
        private final String name;
        private final String unit;
        private final DoubleSupplier inUse;
        private final DoubleSupplier liveEstimate;
        private final IntSupplier sampleCount;
        private final double thresholdFraction;
        private final int minSamples;

        private Watch(String name, String unit, DoubleSupplier inUse, DoubleSupplier liveEstimate,
                IntSupplier sampleCount, double thresholdFraction, int minSamples) {
            this.name = name;
            this.unit = unit;
            this.inUse = inUse;
            this.liveEstimate = liveEstimate;
            this.sampleCount = sampleCount;
            this.thresholdFraction = thresholdFraction;
            this.minSamples = minSamples;
        }

        /** @return the value the robot is currently running on. */
        public double getInUse() {
            return inUse.getAsDouble();
        }

        /** @return the current live estimate. */
        public double getLiveEstimate() {
            return liveEstimate.getAsDouble();
        }

        /** @return how many observations the estimate rests on. */
        public int getSamples() {
            return sampleCount.getAsInt();
        }

        /**
         * @return signed difference as a fraction of the value in use. Positive means the estimate
         *     is larger than what the robot is using.
         */
        public double getDriftFraction() {
            double current = getInUse();
            if (current == 0) {
                return 0;
            }
            return getLiveEstimate() / current - 1.0;
        }

        /**
         * @return true when there is enough evidence to take the estimate seriously. Without this
         *     an estimate built from three samples would raise an alarm on noise alone.
         */
        public boolean hasEnoughEvidence() {
            return getSamples() >= minSamples;
        }

        /** @return true when the estimate has drifted past its threshold, with enough evidence. */
        public boolean hasDrifted() {
            return hasEnoughEvidence() && Math.abs(getDriftFraction()) > thresholdFraction;
        }

        /** @return true when this value may be adopted without a human. */
        public boolean mayAutoAdopt() {
            return CalibrationStore.mayAutoAdopt(name);
        }

        public String getName() {
            return name;
        }

        public String getUnit() {
            return unit;
        }

        public double getThresholdFraction() {
            return thresholdFraction;
        }
    }

    private final List<Watch> watches = new ArrayList<>();

    /**
     * Watches a quantity.
     *
     * @param name              Stable name, matching the {@link CalibrationStore} key.
     * @param unit              For the report, e.g. "m" or "V/(m/s)".
     * @param inUse             The value the robot is running on right now.
     * @param liveEstimate      The current best estimate from live data.
     * @param sampleCount       How many observations back the estimate.
     * @param thresholdFraction Drift beyond this fraction is worth reporting, e.g. 0.01 for 1%.
     * @param minSamples        Evidence needed before the estimate is believed at all.
     * @return the registered watch, for direct inspection.
     */
    public Watch watch(String name, String unit, DoubleSupplier inUse, DoubleSupplier liveEstimate,
            IntSupplier sampleCount, double thresholdFraction, int minSamples) {
        Watch registered = new Watch(
                name, unit, inUse, liveEstimate, sampleCount, thresholdFraction, minSamples);
        watches.add(registered);
        return registered;
    }

    /** Publishes every watch. Call once per loop. */
    public void update() {
        boolean anyDrifted = false;
        int driftedCount = 0;

        for (Watch watch : watches) {
            String root = "Calibration/Drift/" + watch.getName();
            Logger.recordOutput(root + "/InUse", watch.getInUse());
            Logger.recordOutput(root + "/LiveEstimate", watch.getLiveEstimate());
            Logger.recordOutput(root + "/DriftPercent", watch.getDriftFraction() * 100);
            Logger.recordOutput(root + "/Samples", watch.getSamples());
            Logger.recordOutput(root + "/EnoughEvidence", watch.hasEnoughEvidence());
            Logger.recordOutput(root + "/Drifted", watch.hasDrifted());

            if (watch.hasDrifted()) {
                anyDrifted = true;
                driftedCount++;
            }
        }

        Logger.recordOutput("Calibration/Drift/AnyDrifted", anyDrifted);
        Logger.recordOutput("Calibration/Drift/DriftedCount", driftedCount);
        Logger.recordOutput("Calibration/Drift/WatchCount", watches.size());
    }

    /** @return every watch that has drifted past its threshold with enough evidence. */
    public List<Watch> getDrifted() {
        List<Watch> drifted = new ArrayList<>();
        for (Watch watch : watches) {
            if (watch.hasDrifted()) {
                drifted.add(watch);
            }
        }
        return drifted;
    }

    /** @return every registered watch. */
    public List<Watch> getWatches() {
        return List.copyOf(watches);
    }

    /** Removes every watch. Tests only; the robot registers its set once. */
    public void clear() {
        watches.clear();
    }

    /**
     * Prints what has drifted and what to do about it.
     *
     * <p>Separates the values that may be adopted automatically from those needing a decision, so
     * the report is directly actionable rather than a list of numbers.
     */
    public void printReport() {
        System.out.println("=====================================================");
        System.out.println(" CALIBRATION DRIFT");

        if (watches.isEmpty()) {
            System.out.println(" nothing being watched");
            System.out.println("=====================================================");
            return;
        }

        for (Watch watch : watches) {
            String status;
            if (!watch.hasEnoughEvidence()) {
                status = String.format("insufficient evidence (%d samples)", watch.getSamples());
            } else if (watch.hasDrifted()) {
                status = String.format("DRIFTED %+.2f%% (threshold %.2f%%)",
                        watch.getDriftFraction() * 100, watch.getThresholdFraction() * 100);
            } else {
                status = String.format("stable %+.2f%%", watch.getDriftFraction() * 100);
            }

            System.out.printf("   %-28s in use %.6f %s, live %.6f — %s%n",
                    watch.getName(), watch.getInUse(), watch.getUnit(),
                    watch.getLiveEstimate(), status);
        }

        List<Watch> drifted = getDrifted();
        if (!drifted.isEmpty()) {
            System.out.println("-----------------------------------------------------");
            System.out.println(" To adopt these, run the promotion command from Test mode.");
            for (Watch watch : drifted) {
                System.out.printf("   %-28s %s%n", watch.getName(),
                        watch.mayAutoAdopt()
                                ? "safe to adopt automatically"
                                : "NEEDS A HUMAN — measured against the thing it determines");
            }
        }

        System.out.println("=====================================================");
    }
}
