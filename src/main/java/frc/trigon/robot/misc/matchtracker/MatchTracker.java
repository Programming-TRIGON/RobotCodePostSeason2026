package frc.trigon.robot.misc.matchtracker;

import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.lib.utilities.flippable.Flippable;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class MatchTracker {
    private static final LoggedNetworkBoolean hubActiveOverride = new LoggedNetworkBoolean("MatchTracker/HubActiveOverride", false);

    public static void logInfo() {
        Logger.recordOutput("MatchTracker/IsHubActive", isHubActive());
        Logger.recordOutput("MatchTracker/TimeUntilNextShift", getTimeUntilNextShift());
        Logger.recordOutput("MatchTracker/SecondsLeftInMatch", getCurrentMatchTimeSeconds());
    }

    @AutoLogOutput(key = "MatchTracker/IsHubActive")
    public static boolean isHubActive() {
        boolean isRedAlliance = Flippable.isRedAlliance();
        double currentMatchTimeSeconds = getCurrentMatchTimeSeconds();
        char autoWinner = getAutoWinner();

        if (hubActiveOverride.get()) {
            return true;
        }

        if (DriverStation.isAutonomousEnabled())
            return true;

        if (autoWinner != 'R' && autoWinner != 'B') {
            return false;
        }

        if (currentMatchTimeSeconds < MatchTrackerConstants.END_GAME_SHIFT_START_TIME_SECONDS)
            return true;

        if (autoWinner == 'R') {
            if (currentMatchTimeSeconds < MatchTrackerConstants.FOURTH_SHIFT_START_TIME_SECONDS + MatchTrackerConstants.HUB_ACTIVATION_EARLY_SECONDS) {
                return isRedAlliance;
            }

            if (currentMatchTimeSeconds < MatchTrackerConstants.THIRD_SHIFT_START_TIME_SECONDS + MatchTrackerConstants.HUB_ACTIVATION_EARLY_SECONDS) {
                return !isRedAlliance;
            }

            if (currentMatchTimeSeconds < MatchTrackerConstants.SECOND_SHIFT_START_TIME_SECONDS + MatchTrackerConstants.HUB_ACTIVATION_EARLY_SECONDS) {
                return isRedAlliance;
            }

            if (currentMatchTimeSeconds < MatchTrackerConstants.FIRST_SHIFT_START_TIME_SECONDS + MatchTrackerConstants.HUB_ACTIVATION_EARLY_SECONDS) {
                return !isRedAlliance;
            }
        }

        if (autoWinner == 'B') {
            if (currentMatchTimeSeconds < MatchTrackerConstants.FOURTH_SHIFT_START_TIME_SECONDS + MatchTrackerConstants.HUB_ACTIVATION_EARLY_SECONDS) {
                return !isRedAlliance;
            }

            if (currentMatchTimeSeconds < MatchTrackerConstants.THIRD_SHIFT_START_TIME_SECONDS + MatchTrackerConstants.HUB_ACTIVATION_EARLY_SECONDS) {
                return isRedAlliance;
            }

            if (currentMatchTimeSeconds < MatchTrackerConstants.SECOND_SHIFT_START_TIME_SECONDS + MatchTrackerConstants.HUB_ACTIVATION_EARLY_SECONDS) {
                return !isRedAlliance;
            }

            if (currentMatchTimeSeconds < MatchTrackerConstants.FIRST_SHIFT_START_TIME_SECONDS + MatchTrackerConstants.HUB_ACTIVATION_EARLY_SECONDS) {
                return isRedAlliance;
            }
        }

        return currentMatchTimeSeconds < MatchTrackerConstants.TRANSITION_SHIFT_START_TIME_SECONDS;
    }

    private static double getTimeUntilNextShift() {
        double currentMatchTimeSeconds = getCurrentMatchTimeSeconds();

        if (DriverStation.isAutonomousEnabled()) {
            return currentMatchTimeSeconds;
        }

        if (!DriverStation.isTeleopEnabled()) {
            return 0;
        }

        if (currentMatchTimeSeconds > MatchTrackerConstants.TRANSITION_SHIFT_START_TIME_SECONDS) {
            return currentMatchTimeSeconds - MatchTrackerConstants.TRANSITION_SHIFT_START_TIME_SECONDS;
        }

        if (currentMatchTimeSeconds > MatchTrackerConstants.FIRST_SHIFT_START_TIME_SECONDS) {
            return currentMatchTimeSeconds - MatchTrackerConstants.FIRST_SHIFT_START_TIME_SECONDS;
        }

        if (currentMatchTimeSeconds > MatchTrackerConstants.SECOND_SHIFT_START_TIME_SECONDS) {
            return currentMatchTimeSeconds - MatchTrackerConstants.SECOND_SHIFT_START_TIME_SECONDS;
        }

        if (currentMatchTimeSeconds > MatchTrackerConstants.THIRD_SHIFT_START_TIME_SECONDS) {
            return currentMatchTimeSeconds - MatchTrackerConstants.THIRD_SHIFT_START_TIME_SECONDS;
        }

        if (currentMatchTimeSeconds > MatchTrackerConstants.FOURTH_SHIFT_START_TIME_SECONDS) {
            return currentMatchTimeSeconds - MatchTrackerConstants.FOURTH_SHIFT_START_TIME_SECONDS;
        }

        if (currentMatchTimeSeconds > MatchTrackerConstants.END_GAME_SHIFT_START_TIME_SECONDS) {
            return currentMatchTimeSeconds - MatchTrackerConstants.END_GAME_SHIFT_START_TIME_SECONDS;
        }

        return currentMatchTimeSeconds;
    }

    private static double getCurrentMatchTimeSeconds() {
        return DriverStation.getMatchTime();
    }

    private static char getAutoWinner() {
        String gameData = DriverStation.getGameSpecificMessage();

        if (!gameData.isEmpty()) {
            char autoWinner = gameData.charAt(0);

            if (autoWinner == 'R' || autoWinner == 'B') {
                return autoWinner;
            }
        }

        return 0;
    }
}
