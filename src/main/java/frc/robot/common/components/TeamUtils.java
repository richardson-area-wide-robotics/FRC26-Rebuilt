package frc.robot.common.components;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.RobotBase;
import org.json.JSONObject;

public class TeamUtils {


    /**
     * Helper method to get the team number, the same as {@link HALUtil#getTeamNumber}
     * Only added because I can never remember the import
     *
     * @author Hudson Strub
     * @since 2025
     */
    public static int getTeamNumber() {
        if (RobotBase.isSimulation()) {
            // Override in sim since HALUtil returns 0
            return getOfflineTeamNumber(); // e.g. 1745
        }
        return HALUtil.getTeamNumber();
    }

    /**
     * Helper method to get the team number from the wpilib_preferences.json
     *
     * @author Hudson Strub
     * @since 2025 Offseason
     */
    public static int getOfflineTeamNumber() {
            try {
                // Look inside the project’s .wpilib folder
                Path projectDir = Paths.get(System.getProperty("user.dir"));
                Path prefPath = projectDir.resolve(".wpilib").resolve("wpilib_preferences.json");

                if (Files.exists(prefPath)) {
                    String content = Files.readString(prefPath);
                    JSONObject json = new JSONObject(content);
                    return json.optInt("teamNumber", 0);
                }
            } catch (IOException e) {
                System.err.println("Error loading team number from file!");
            }
            return 200; // fallback in sim if file missing
        }
    }
