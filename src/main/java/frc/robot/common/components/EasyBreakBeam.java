package frc.robot.common.components;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Wrapper for making break beams
 *
 * @author Hudson Strub
 * @since 2025 Offseason
 */
public record EasyBreakBeam(DigitalInput input) {
    public EasyBreakBeam(int channel) {
        this(new DigitalInput(channel));
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
