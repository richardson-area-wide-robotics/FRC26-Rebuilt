package frc.robot.pearce.components;

import edu.wpi.first.wpilibj.DriverStation;

public class HubStatus {

    /** Alliance whose HUB goes inactive first according to FMS */
    public enum AllianceGoalInactive {
        RED, BLUE, UNKNOWN_CORRUPT_DATA, UNKNOWN_NO_DATA
    }

    /** Current HUB state */
    public enum HubState {
        ACTIVE, INACTIVE
    }

    /** Returns the raw game data string sent by the FMS. */
    public static String getGameData() {
        return DriverStation.getGameSpecificMessage();
    }

    /** Returns which alliance's HUB goes inactive first according to FMS. */
    public static AllianceGoalInactive getFirstInactiveAlliance() {
        String gameData = getGameData();
        if (gameData.isEmpty()) {
            return AllianceGoalInactive.UNKNOWN_NO_DATA; // Data hasn't been sent yet
        }
        return switch (gameData.charAt(0)) {
            case 'R' -> AllianceGoalInactive.RED;
            case 'B' -> AllianceGoalInactive.BLUE;
            default -> AllianceGoalInactive.UNKNOWN_CORRUPT_DATA;
        };
    }

    /** Returns the current HUB status for the given alliance. */
    public static HubState getHubStatus(DriverStation.Alliance alliance, double matchTime) {
        AllianceGoalInactive firstInactive = getFirstInactiveAlliance();

        // AUTO: 150–130 sec remaining, END GAME: 30–0 sec remaining
        if ((matchTime > 130 && matchTime <= 150) || (matchTime >= 0 && matchTime <= 30)) {
            return HubState.ACTIVE;
        }

        // Determine shift number
        int shift = getShift(matchTime);
        if (shift == 0) { // Transition shift
            return HubState.ACTIVE;
        }

        // Determine if this alliance is inactive this shift
        boolean inactiveThisShift = isInactiveThisShift(alliance, firstInactive, shift);
        return inactiveThisShift ? HubState.INACTIVE : HubState.ACTIVE;
    }

    /** Returns both alliance HUB statuses for convenience. */
    public static HubState[] getBothHubStatuses(double matchTime) {
        return new HubState[]{
                getHubStatus(DriverStation.Alliance.Red, matchTime),
                getHubStatus(DriverStation.Alliance.Blue, matchTime)
        };
    }

    /** Determines if the given alliance is inactive for this shift. */
    private static boolean isInactiveThisShift(DriverStation.Alliance alliance, AllianceGoalInactive firstInactive, int shift) {
        boolean isFirstInactiveAlliance =
                (alliance == DriverStation.Alliance.Red && firstInactive == AllianceGoalInactive.RED) ||
                        (alliance == DriverStation.Alliance.Blue && firstInactive == AllianceGoalInactive.BLUE);

        // Shift alternates:
        // Odd shifts: firstInactive is INACTIVE
        // Even shifts: firstInactive is ACTIVE
        return (shift % 2 == 1) ? isFirstInactiveAlliance : !isFirstInactiveAlliance;
    }

    /** Converts match time to the current ALLIANCE SHIFT number (1–4). Returns 0 for transition shift. */
    private static int getShift(double matchTime) {
        if (matchTime > 130 && matchTime <= 150) return 0;   // AUTO handled separately
        if (matchTime > 125) return 0;   // Transition shift
        if (matchTime > 105) return 1;   // Shift 1
        if (matchTime > 80) return 2;    // Shift 2
        if (matchTime > 55) return 3;    // Shift 3
        if (matchTime > 30) return 4;    // Shift 4
        return 0; // End Game handled separately
    }
}
