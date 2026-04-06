package frc.robot.common.components.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.common.annotations.DashboardVariable;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

/**
 * Automatically updates SmartDashboard for any field annotated with @DashboardVariable.
 * Supports both static and registered instance fields.
 *
 * @author Hudson Strub
 * @since 2025 Offseason
 */
public class DashboardAutoUpdater {

    private record DashboardEntry(Field field, DashboardVariable annotation, String key, Object instance) {}

    private static final List<DashboardEntry> entries = new ArrayList<>();

    static {
        // Static scanning removed to improve boot-up performance. 
        // Use DashboardAutoUpdater.register(this) in constructors instead.
    }

    /**
     * Register an object to track its @DashboardVariable fields.
     */
    public static void register(Object instance) {
        Class<?> clazz = instance.getClass();
        for (Field field : clazz.getDeclaredFields()) {
            if (!field.isAnnotationPresent(DashboardVariable.class)) {
                continue;
            }

            try {
                if (java.lang.reflect.Modifier.isStatic(field.getModifiers())) {
                    System.out.println("Skipping static field in instance registration: "
                            + clazz.getSimpleName() + "." + field.getName());
                    continue;
                }

                field.setAccessible(true);
                DashboardVariable annotation = field.getAnnotation(DashboardVariable.class);
                String key = annotation.name().isEmpty() ? field.getName() : annotation.name();

                System.out.println("Registered instance DashboardVariable: "
                        + clazz.getSimpleName() + "." + field.getName() + " → " + key);

                entries.add(new DashboardEntry(field, annotation, key, instance));

                if (annotation.persistent()) {
                    SmartDashboard.setPersistent(key);
                }

            } catch (Exception e) {
                System.err.println("Error registering instance of " + instance);
            }
        }
    }

    /**
     * Update everything to the
     */
    public static void updateAll() {
        for (DashboardEntry entry : entries) {
            try {
                Object value = entry.field.get(entry.instance); // instance = null → static

                if (value instanceof Number) {
                    SmartDashboard.putNumber(entry.key, ((Number) value).doubleValue());
                } else if (value instanceof Boolean) {
                    SmartDashboard.putBoolean(entry.key, (Boolean) value);
                } else if (value != null) {
                    SmartDashboard.putString(entry.key, value.toString());
                }

            } catch (Exception e) {
                System.err.println("Error updating dashboard! " + e.getMessage());
            }
        }
    }
}
