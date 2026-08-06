package frc.robot.common.components.diagnostics;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.function.BooleanSupplier;

/**
 * The guided calibration's four buttons, from a controller <em>or</em> a dashboard <em>or</em> a
 * keyboard.
 *
 * <p>A gamepad is the wrong input for some of these steps. The operator holding the arm has no free
 * hand, whoever is squaring the robot against a wall is nowhere near the driver station, and the
 * person reading the assessments is at the laptop looking at the console. So each button is the OR of
 * a controller trigger and a NetworkTables boolean, and either one works at any time.
 *
 * <h2>Why the NetworkTables entries clear themselves</h2>
 *
 * <p>A dashboard toggle <b>stays true</b> once clicked, and a script that sets one has no natural
 * moment to unset it. Left alone that produces a button held down for ever, which
 * {@link PressLatch} correctly treats as a single press and then ignores &mdash; so the second press
 * never arrives and the routine appears to hang.
 *
 * <p>So reading one that is true consumes it: the robot writes false straight back. That turns any
 * widget, toggle or plain {@code set} into a momentary press, and it means the writer never has to
 * think about releasing. It also makes the entry self-describing on a dashboard: it sits at false and
 * blinks true, so you can see the robot receiving each press.
 *
 * <h2>Keyboard</h2>
 *
 * <p>There is no keyboard input on a real driver station &mdash; the WPILib simulator maps keys to
 * joysticks, but the FRC Driver Station does not. {@code tools/calib_keys.py} closes that gap from the
 * other side: it reads single keypresses on the laptop and sets these same entries, so a keyboard
 * works without the robot knowing anything about keyboards.
 */
public final class CalibrationButtons {

    /** Table the four booleans live in. Short, because a human types this into a dashboard. */
    public static final String TABLE = "Calibration";

    private final BooleanEntry run;
    private final BooleanEntry next;
    private final BooleanEntry previous;
    private final BooleanEntry skip;

    /** Creates the four entries and publishes them as false. */
    public CalibrationButtons() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(TABLE);

        run = entry(table, "Run");
        next = entry(table, "Next");
        previous = entry(table, "Previous");
        skip = entry(table, "Skip");
    }

    private static BooleanEntry entry(NetworkTable table, String name) {
        BooleanEntry created = table.getBooleanTopic(name).getEntry(false);
        // Published up front so the four appear on a dashboard before anything is pressed. A widget
        // cannot be bound to a topic that does not exist yet, so without this the operator has to run
        // the routine once, find the entries, then restart to bind them.
        created.set(false);
        return created;
    }

    /**
     * Reads one entry and consumes it.
     *
     * @param entry The entry.
     * @return whether it was set since the last read.
     */
    private static boolean consume(BooleanEntry entry) {
        if (!entry.get()) {
            return false;
        }
        entry.set(false);
        return true;
    }

    /**
     * Combines a controller trigger with the matching dashboard entry.
     *
     * @param controller Reads true while the controller button is held.
     * @param entry      The dashboard entry.
     * @return a supplier true when either has fired.
     */
    private static BooleanSupplier either(BooleanSupplier controller, BooleanEntry entry) {
        // The controller is read first and NOT short-circuited past, because consume() has a side
        // effect: skipping it while a controller button happened to be held would leave a dashboard
        // press latched, to fire confusingly later.
        return () -> {
            boolean dashboard = consume(entry);
            return controller.getAsBoolean() || dashboard;
        };
    }

    /**
     * @param controller Controller trigger for RUN.
     * @return the RUN button.
     */
    public BooleanSupplier runButton(BooleanSupplier controller) {
        return either(controller, run);
    }

    /**
     * @param controller Controller trigger for NEXT.
     * @return the NEXT button.
     */
    public BooleanSupplier nextButton(BooleanSupplier controller) {
        return either(controller, next);
    }

    /**
     * @param controller Controller trigger for PREVIOUS.
     * @return the PREVIOUS button.
     */
    public BooleanSupplier previousButton(BooleanSupplier controller) {
        return either(controller, previous);
    }

    /**
     * @param controller Controller trigger for SKIP.
     * @return the SKIP button.
     */
    public BooleanSupplier skipButton(BooleanSupplier controller) {
        return either(controller, skip);
    }

    /**
     * Clears all four.
     *
     * <p>Called when a routine starts, so a press made while nothing was listening cannot fire
     * immediately &mdash; which would consume the first step's prompt before anyone had read it.
     */
    public void clearAll() {
        run.set(false);
        next.set(false);
        previous.set(false);
        skip.set(false);
    }
}
