import frc.robot.common.DefaultContainer;
import frc.robot.common.components.RobotContainerRegistry;
import frc.robot.common.interfaces.IRobotContainer;
import org.junit.jupiter.api.Test;

import java.lang.reflect.Field;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class RobotContainerRegistryTest {

    @Test
    void testUnknownTeamReturnsDefault() {
        IRobotContainer container = RobotContainerRegistry.createContainerForTeam(9999);

        assertNotNull(container, "Unknown team should still return a container");
        assertEquals(
                DefaultContainer.class,
                container.getClass(),
                "Unknown team should return DefaultContainer"
        );
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