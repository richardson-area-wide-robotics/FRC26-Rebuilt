package frc.robot.rebuilt.components;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

/**
 * Single owner of everything the robot knows about the field: which alliance we are, whether
 * our HUB is currently scoring, and what the FMS game-specific message says.
 *
 * <p>This replaces a set of public mutable statics on the container that were only refreshed
 * inside {@code teleopPeriodic()}. Two problems followed from that: the hub interlock was
 * stale for the whole of autonomous, and every read went through
 * {@code DriverStation.getAlliance().get()} with no presence check, which throws before the
 * driver station has reported an alliance.
 *
 * <p>{@link #update()} is called from {@code robotPeriodic()}, so this is live in every mode.
 * Until an alliance is known, {@link #isHubActive()} returns {@code true} — a robot that
 * cannot tell whose hub is open should still be able to shoot rather than sit inert.
 */
public class FieldState {

    private Optional<DriverStation.Alliance> alliance = Optional.empty();
    private HubStatus.HubState ourHubState = HubStatus.HubState.ACTIVE;
    private HubStatus.HubState redHubState = HubStatus.HubState.ACTIVE;
    private HubStatus.HubState blueHubState = HubStatus.HubState.ACTIVE;
    private double matchTime;

    /** Refreshes all cached field state. Call once per robot loop, in every mode. */
    public void update() {
        alliance = DriverStation.getAlliance();
        matchTime = DriverStation.getMatchTime();

        HubStatus.HubState[] both = HubStatus.getBothHubStatuses(matchTime);
        redHubState = both[0];
        blueHubState = both[1];

        if (alliance.isPresent()) {
            ourHubState = alliance.get() == DriverStation.Alliance.Red ? redHubState : blueHubState;
        } else {
            // No alliance reported yet. Assume our goal is open rather than locking the
            // shooter out entirely.
            ourHubState = HubStatus.HubState.ACTIVE;
        }

        log();
    }

    private void log() {
        Logger.recordOutput("Field/HasAlliance", alliance.isPresent());
        Logger.recordOutput("Field/IsRed", isAllianceRed());
        Logger.recordOutput("Field/MatchTime", matchTime);
        Logger.recordOutput("Field/OurHubState", ourHubState.name());
        Logger.recordOutput("Field/RedHubState", redHubState.name());
        Logger.recordOutput("Field/BlueHubState", blueHubState.name());
        Logger.recordOutput("Field/HubActive", isHubActive());
        Logger.recordOutput("Field/HubBlinking", isHubBlinking());
        Logger.recordOutput("Field/GameData", HubStatus.getGameData());
        Logger.recordOutput("Field/FirstInactiveAlliance",
                HubStatus.getFirstInactiveAlliance().name());
    }

    /** @return true once the driver station has told us which alliance we are. */
    public boolean hasAlliance() {
        return alliance.isPresent();
    }

    /** @return true when we are on the red alliance. False if red-vs-blue is not yet known. */
    public boolean isAllianceRed() {
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    /** @return our alliance, if the driver station has reported one. */
    public Optional<DriverStation.Alliance> getAlliance() {
        return alliance;
    }

    /** @return true while our alliance's hub is scoring, so the shooter may spin up. */
    public boolean isHubActive() {
        return ourHubState == HubStatus.HubState.ACTIVE;
    }

    /** @return true while our hub is mid-transition, i.e. about to change state. */
    public boolean isHubBlinking() {
        return ourHubState == HubStatus.HubState.BLINKING;
    }

    /** @return our hub's current state. */
    public HubStatus.HubState getOurHubState() {
        return ourHubState;
    }

    /**
     * Whether the FMS game-specific message names our alliance.
     *
     * <p><strong>Semantics need confirming against the game manual.</strong> The original
     * code read this byte two different ways in two different places: {@code HubStatus}
     * treats it as "the alliance whose HUB goes inactive first", while the container treated
     * the same byte as "the alliance that won autonomous". Only one can be right.
     *
     * <p>This method reports the raw fact — the message names us — without claiming what it
     * means. The old container version could never return true regardless, because it
     * compared strings with {@code ==} and used the wrong letter case.
     *
     * @return true when the FMS message's alliance matches ours.
     */
    public boolean gameDataNamesUs() {
        HubStatus.AllianceGoalInactive named = HubStatus.getFirstInactiveAlliance();
        if (!alliance.isPresent()) {
            return false;
        }
        return switch (named) {
            case RED -> alliance.get() == DriverStation.Alliance.Red;
            case BLUE -> alliance.get() == DriverStation.Alliance.Blue;
            case UNKNOWN_CORRUPT_DATA, UNKNOWN_NO_DATA -> false;
        };
    }
}
