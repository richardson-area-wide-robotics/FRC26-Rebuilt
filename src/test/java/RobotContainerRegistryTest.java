import frc.robot.common.container.DefaultContainer;
import frc.robot.common.components.RobotContainerRegistry;
import frc.robot.common.container.IRobotContainer;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Field;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class RobotContainerRegistryTest {

    @Test
    void testUnknownTeamsReturnDefault() {
        int[] unknownTeams = {0, 1, 123, 9999, -5, 100000};

        for (int team : unknownTeams) {
            IRobotContainer container = RobotContainerRegistry.createContainerForTeam(team);

            assertEquals(DefaultContainer.class, container.getClass(),
                    "Team " + team + " should return DefaultContainer");
        }
    }

    @Test
    void testAllContainersImplementInterface() throws Exception {
        Field field = RobotContainerRegistry.class.getDeclaredField("TEAM_CONTAINERS");
        field.setAccessible(true);

        Map<Integer, Class<?>> map =
                (Map<Integer, Class<?>>) field.get(null);

        for (Class<?> clazz : map.values()) {
            assertTrue(IRobotContainer.class.isAssignableFrom(clazz));
        }
    }

    @Test
    void testRegisteredContainersHaveCreateContainerMethod() throws Exception {
        Field field = RobotContainerRegistry.class.getDeclaredField("TEAM_CONTAINERS");
        field.setAccessible(true);

        Map<Integer, Class<?>> map =
                (Map<Integer, Class<?>>) field.get(null);

        for (Class<?> clazz : map.values()) {
            assertDoesNotThrow(() -> clazz.getMethod("createContainer"));
        }
    }

    @Test
    void testTeam1745IsRegistered() throws Exception {
        Field field = RobotContainerRegistry.class.getDeclaredField("TEAM_CONTAINERS");
        field.setAccessible(true);

        Map<Integer, Class<?>> map =
                (Map<Integer, Class<?>>) field.get(null);

        assertTrue(map.containsKey(1745), "1745 should be registered");
    }
}