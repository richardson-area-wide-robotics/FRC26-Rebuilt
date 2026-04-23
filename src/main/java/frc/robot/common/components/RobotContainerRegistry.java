package frc.robot.common.components;

import com.strubium.ssjprofiler.Profiler;
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
    private static final Map<Integer, Class<?>> TEAM_CONTAINERS = new HashMap<>();

    static {
        Profiler teamContainerScan = new Profiler("team container scan");
        teamContainerScan.start();

        // Scan for classes annotated with @Robot in the frc.robot package
        Reflections reflections = new Reflections("frc.robot");
        Set<Class<?>> annotatedClasses = reflections.getTypesAnnotatedWith(Robot.class);

        for (Class<?> clazz : annotatedClasses) {
            try {
                clazz.getMethod("createContainer");
            } catch (NoSuchMethodException e) {
                throw new IllegalStateException( //Crash early if we don't have the required createContainer method
                        clazz.getName() + " is annotated with @Robot but does not define " +
                                "public static IRobotContainer createContainer()"
                );
            }

            Robot annotation = clazz.getAnnotation(Robot.class);
            TEAM_CONTAINERS.put(annotation.team(), clazz);
        }
        teamContainerScan.end();
    }

    public static IRobotContainer createContainerForTeam(int teamNumber) {
        Class<?> containerClass = TEAM_CONTAINERS.get(teamNumber);

        if (containerClass == null) {
            System.out.println("We can't find a container for " + teamNumber + "! Using default");
            return DefaultContainer.createContainer();
        }

        try {
            return (IRobotContainer) containerClass.getMethod("createContainer").invoke(null);
        } catch (Exception e) {
            System.err.println("Error creating container for " + teamNumber + "!");
            e.printStackTrace();
        }

        return null;
    }
}
