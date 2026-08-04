package frc.robot.common.components.diagnostics;

/**
 * The fit the SysId desktop analyser performs, done on the robot instead.
 *
 * <p>SysId's model for a drivetrain is a straight line in three regressors:
 *
 * <pre>
 *   V = kS * sgn(v)  +  kV * v  +  kA * a
 * </pre>
 *
 * <p>Given samples of applied voltage, velocity and acceleration, recovering kS, kV and kA is an
 * ordinary least squares problem. That is all the analyser GUI does with the log you hand it, so
 * doing it here removes the whole offline step: run the routine, read the gains off the console.
 *
 * <h2>Why this exists alongside {@link DriveCharacterization.FeedforwardFit}</h2>
 *
 * <p>That one fits two parameters from a step-and-settle sweep. Steady state means acceleration is
 * zero by construction, so <b>kA is not merely unmeasured there, it is unmeasurable</b> — there is no
 * information about it in the data. kA is what makes a velocity setpoint reachable promptly rather
 * than eventually, so it matters for path following.
 *
 * <p>The two are worth keeping side by side as a cross-check. They use different data and different
 * arithmetic, so kS and kV agreeing between them is real evidence; disagreeing means one of the runs
 * was bad, and it is much better to learn that from two numbers than from a robot that drives oddly.
 *
 * <h2>What this does not replace</h2>
 *
 * <p>The desktop analyser also plots residuals, lets you trim the time window by hand, and shows the
 * data so a human can spot a run where the wheels slipped or the robot hit something. This produces
 * a number and an R². <b>Treat a low R² as "go and look at the log", not as a gain to paste.</b>
 */
public final class SysIdRegression {

    private SysIdRegression() {
    }

    /**
     * Velocity below which a sample is discarded, in the same unit as the samples.
     *
     * <p>{@code sgn(v)} is meaningless around zero and static friction dominates there, so those
     * samples carry no usable information about kS and actively corrupt it. The desktop analyser has
     * the same control, exposed as its velocity threshold.
     */
    public static final double DEFAULT_VELOCITY_THRESHOLD = 0.05;

    /** A fitted feedforward, with the diagnostics needed to decide whether to trust it. */
    public record Gains(double kS, double kV, double kA, double rSquared, int samples,
            boolean wellConditioned) {

        /**
         * @return true when the fit is worth pasting.
         *
         *     <p>Requires enough samples, a good R², a well conditioned system, and physically
         *     sensible signs. A negative kV means voltage and velocity moved in opposite directions,
         *     which is not a bad fit — it is a wiring or sign error, and no amount of extra data
         *     will improve it.
         */
        public boolean isTrustworthy() {
            return samples >= 50
                    && rSquared >= 0.95
                    && wellConditioned
                    && kV > 0
                    && kA >= 0
                    && kS >= 0;
        }

        /** @return a readable summary, paste-ready when trustworthy. */
        public String describe() {
            if (!wellConditioned) {
                return String.format(
                        "SINGULAR — the three regressors were not independent over %d samples. "
                                + "Usually means the dynamic test never ran, so acceleration was "
                                + "always near zero and kA cannot be separated from the rest.",
                        samples);
            }

            String verdict;
            if (isTrustworthy()) {
                verdict = "OK";
            } else if (kV <= 0) {
                verdict = "REJECT: kV is not positive — voltage and velocity disagree in sign, "
                        + "which is a wiring or inversion fault, not a fit problem";
            } else if (kA < 0) {
                verdict = "REJECT: kA is negative, which is unphysical — check the dynamic run for "
                        + "wheel slip or a collision";
            } else if (kS < 0) {
                verdict = "REJECT: kS is negative, which is unphysical";
            } else if (samples < 50) {
                verdict = "REJECT: only " + samples + " samples, need 50";
            } else {
                verdict = String.format("SUSPECT: R2 %.4f is below 0.95 — look at the log before "
                        + "using these", rSquared);
            }

            return String.format("kS = %.4f  kV = %.4f  kA = %.4f   (R2 %.4f, %d samples) — %s",
                    kS, kV, kA, rSquared, samples, verdict);
        }
    }

    /**
     * Accumulates samples and fits on demand.
     *
     * <p>Keeps only the six sums of products the normal equations need rather than the samples
     * themselves, so a long run costs nothing in memory. The consequence is that the time window
     * cannot be trimmed after the fact the way the desktop analyser allows — decide what to feed it
     * as it arrives.
     */
    public static final class Accumulator {

        private final double velocityThreshold;

        // Upper triangle of X'X, where the regressors are s = sgn(v), v and a.
        private double sumSS;
        private double sumSV;
        private double sumSA;
        private double sumVV;
        private double sumVA;
        private double sumAA;

        // X'y
        private double sumSY;
        private double sumVY;
        private double sumAY;

        // For R².
        private double sumY;
        private double sumYY;

        private int samples;

        /** Uses {@link #DEFAULT_VELOCITY_THRESHOLD}. */
        public Accumulator() {
            this(DEFAULT_VELOCITY_THRESHOLD);
        }

        /**
         * @param velocityThreshold Samples slower than this are discarded.
         */
        public Accumulator(double velocityThreshold) {
            this.velocityThreshold = Math.abs(velocityThreshold);
        }

        /**
         * Folds in one sample.
         *
         * @param volts        Applied voltage.
         * @param velocity     Measured velocity.
         * @param acceleration Measured acceleration, same distance unit per second squared.
         * @return true if the sample was used rather than discarded as too slow.
         */
        public boolean add(double volts, double velocity, double acceleration) {
            if (Math.abs(velocity) < velocityThreshold) {
                return false;
            }

            double s = Math.signum(velocity);

            sumSS += s * s;
            sumSV += s * velocity;
            sumSA += s * acceleration;
            sumVV += velocity * velocity;
            sumVA += velocity * acceleration;
            sumAA += acceleration * acceleration;

            sumSY += s * volts;
            sumVY += velocity * volts;
            sumAY += acceleration * volts;

            sumY += volts;
            sumYY += volts * volts;

            samples++;
            return true;
        }

        /** @return samples actually used. */
        public int getSamples() {
            return samples;
        }

        /**
         * Solves for the gains.
         *
         * @return the fit, which may be untrustworthy — check {@link Gains#isTrustworthy()}.
         */
        public Gains fit() {
            if (samples < 3) {
                return new Gains(0, 0, 0, 0, samples, false);
            }

            double[][] a = {
                {sumSS, sumSV, sumSA},
                {sumSV, sumVV, sumVA},
                {sumSA, sumVA, sumAA}
            };
            double[] b = {sumSY, sumVY, sumAY};

            double[] beta = solve3x3(a, b);
            if (beta == null) {
                return new Gains(0, 0, 0, 0, samples, false);
            }

            double kS = beta[0];
            double kV = beta[1];
            double kA = beta[2];

            // R² from the sums, without revisiting the samples. Residual sum of squares expands to
            // y'y - 2*beta'X'y + beta'(X'X)beta, every term of which is already accumulated.
            double betaXty = kS * sumSY + kV * sumVY + kA * sumAY;
            double betaXtXbeta =
                    kS * kS * sumSS + kV * kV * sumVV + kA * kA * sumAA
                            + 2 * (kS * kV * sumSV + kS * kA * sumSA + kV * kA * sumVA);
            double ssRes = sumYY - 2 * betaXty + betaXtXbeta;
            double ssTot = sumYY - (sumY * sumY) / samples;

            double rSquared = ssTot > 1e-12 ? 1.0 - ssRes / ssTot : 0.0;

            // Floating point can push a near-perfect fit a hair past 1, or a hopeless one below 0.
            rSquared = Math.max(0.0, Math.min(1.0, rSquared));

            return new Gains(kS, kV, kA, rSquared, samples, true);
        }

        /** Forgets every sample. */
        public void reset() {
            sumSS = sumSV = sumSA = sumVV = sumVA = sumAA = 0;
            sumSY = sumVY = sumAY = 0;
            sumY = sumYY = 0;
            samples = 0;
        }
    }

    /**
     * Solves a 3x3 system by Gaussian elimination with partial pivoting.
     *
     * <p>Hand-rolled rather than pulled from a matrix library because the failure mode matters more
     * than the arithmetic: a singular system here has a specific physical meaning — most often that
     * the dynamic test never ran, so acceleration was always near zero and kA is not separable — and
     * returning null lets the caller say that rather than reporting a confident wrong number.
     *
     * @param a 3x3 coefficients. Overwritten.
     * @param b Right-hand side. Overwritten.
     * @return the solution, or null if the system is singular to working precision.
     */
    static double[] solve3x3(double[][] a, double[] b) {
        int n = 3;

        for (int col = 0; col < n; col++) {
            int pivot = col;
            for (int row = col + 1; row < n; row++) {
                if (Math.abs(a[row][col]) > Math.abs(a[pivot][col])) {
                    pivot = row;
                }
            }

            if (Math.abs(a[pivot][col]) < 1e-12) {
                return null;
            }

            if (pivot != col) {
                double[] swapRow = a[pivot];
                a[pivot] = a[col];
                a[col] = swapRow;

                double swapB = b[pivot];
                b[pivot] = b[col];
                b[col] = swapB;
            }

            for (int row = col + 1; row < n; row++) {
                double factor = a[row][col] / a[col][col];
                for (int k = col; k < n; k++) {
                    a[row][k] -= factor * a[col][k];
                }
                b[row] -= factor * b[col];
            }
        }

        double[] x = new double[n];
        for (int row = n - 1; row >= 0; row--) {
            double sum = b[row];
            for (int k = row + 1; k < n; k++) {
                sum -= a[row][k] * x[k];
            }
            x[row] = sum / a[row][row];
        }

        return x;
    }
}
