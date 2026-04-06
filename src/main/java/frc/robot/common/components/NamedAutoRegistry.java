package frc.robot.common.components;

import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.common.annotations.NamedAuto;

import edu.wpi.first.wpilibj2.command.Command;

import java.lang.reflect.Method;

/**
 * Manually registers methods annotated with @NamedAuto
 * into PathPlanner's NamedCommands system.
 *
 * @author Hudson Strub
 * @since 2025 Offseason
 */
public class NamedAutoRegistry {

    /**
     * Register @NamedAuto methods on a specific object instance.
     */
    public static void register(Object instance) {
        Class<?> clazz = instance.getClass();
        for (Method method : clazz.getDeclaredMethods()) {
            if (!method.isAnnotationPresent(NamedAuto.class)) {
                continue;
            }

            try {
                if (java.lang.reflect.Modifier.isStatic(method.getModifiers())) {
                    System.out.println("Skipping static method in instance registration: "
                            + clazz.getSimpleName() + "." + method.getName());
                    continue;
                }

                NamedAuto annotation = method.getAnnotation(NamedAuto.class);
                method.setAccessible(true);
                Object result = method.invoke(instance);

                if (!(result instanceof Command)) {
                    System.out.println("@NamedAuto method does not return Command: "
                            + clazz.getSimpleName() + "." + method.getName());
                    continue;
                }

                NamedCommands.registerCommand(annotation.value(), (Command) result);
                System.out.println("Registered instance @NamedAuto: "
                        + clazz.getSimpleName() + "." + method.getName()
                        + " → " + annotation.value());

            } catch (Exception e) {
                System.err.println("Error registering instance of " + instance);
            }
        }
    }
}
