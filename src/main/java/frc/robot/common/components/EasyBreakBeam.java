package frc.robot.common.components;

import edu.wpi.first.wpilibj.DigitalInput;
import lombok.Getter;

/** Wrapper for making break beams
 *
 * @author Hudson Strub
 * @since 2025 Offseason
 */
@Getter
public class EasyBreakBeam {
    private final DigitalInput input;

    public EasyBreakBeam(int channel) {
        this.input = new DigitalInput(channel);
    }

    public EasyBreakBeam(DigitalInput input) {
        this.input = input;
    }

    /**
     * Opposite of {@link #isClear()}. Is the break beam broken?
     *
     * @return true if the beam is broken (object is present)
     */
    public boolean isBroken() {
        return !input.get();
    }

    /**
     * Opposite of {@link #isBroken()}
     *
     * @return true if the beam is unbroken (no object detected)
     */
    public boolean isClear() {
        return input.get();
    }
}
