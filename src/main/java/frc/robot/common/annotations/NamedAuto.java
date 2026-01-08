package frc.robot.common.annotations;

import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.annotation.ElementType;

/**
 * Marks a method to be registered as a NamedCommand in PathPlanner.
 *
 * @author Hudson Strub
 * @since 2025 Offseason
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.METHOD)
public @interface NamedAuto {
    /**
     * The name to register the command under.
     * This must match the name used in PathPlanner autos.
     */
    String value();
}
