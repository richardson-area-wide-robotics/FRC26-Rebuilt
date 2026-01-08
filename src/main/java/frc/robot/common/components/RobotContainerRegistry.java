package frc.robot.common.components;

import frc.robot.common.DefaultContainer;
import frc.robot.common.annotations.Robot;
import frc.robot.common.interfaces.IRobotContainer;
import lombok.experimental.UtilityClass;
import org.reflections.Reflections;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;

@UtilityClass
public class RobotContainerRegistry {

    // A map to hold robot containers by team
    private static final Map<Integer, Class<?>> teamContainers = new HashMap<>();

    static {
        // Scan for classes annotated with @Robot in the frc.robot package
        Reflections reflections = new Reflections("frc.robot");
        Set<Class<?>> annotatedClasses = reflections.getTypesAnnotatedWith(Robot.class);

        for (Class<?> clazz : annotatedClasses) {
            Robot annotation = clazz.getAnnotation(Robot.class);
            teamContainers.put(annotation.team(), clazz);
        }
    }

    public static IRobotContainer createContainerForTeam(int teamNumber) {
        Class<?> containerClass = teamContainers.get(teamNumber);

        if (containerClass == null) {
            System.out.println("We can't find a container for " + teamNumber + "! Using default");
            return DefaultContainer.createContainer();
        }

        try {
            return (IRobotContainer) containerClass.getMethod("createContainer").invoke(null);
        } catch (Exception e) {
            System.err.println("Error creating container for " + teamNumber + "!");
        }

        return null;
    }
}
