package frc.robot.common.components;

import frc.robot.common.DefaultContainer;
import frc.robot.common.interfaces.IRobotContainer;
import frc.robot.rebuilt.RebuiltContainer;
import lombok.experimental.UtilityClass;

import java.util.HashMap;
import java.util.Map;

@UtilityClass
public class RobotContainerRegistry {

    private static final Map<Integer, Class<?>> TEAM_CONTAINERS = new HashMap<>();

    static {
        // Register RebuiltContainer manually to avoid expensive reflection scanning
        TEAM_CONTAINERS.put(1745, RebuiltContainer.class);
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
