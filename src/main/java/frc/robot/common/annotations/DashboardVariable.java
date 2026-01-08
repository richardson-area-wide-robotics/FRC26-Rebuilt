package frc.robot.common.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Marks fields that should be sent to SmartDashboard
 *
 * @author Hudson Strub
 * @since 2025 Offseason
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface DashboardVariable {
    String name() default "";  // Optional: override SmartDashboard key
    boolean persistent() default false;  // Optional: keep value across restarts
}
