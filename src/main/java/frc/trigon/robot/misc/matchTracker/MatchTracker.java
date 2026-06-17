package frc.trigon.robot.misc.matchTracker;

import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.lib.utilities.flippable.Flippable;
import org.littletonrobotics.junction.Logger;

public class MatchTracker {
    private static Character forcedGameData = null;

    public static boolean isHubActiveForShot() {
        final double currentMatchTimeSeconds = DriverStation.getMatchTime();
        final double fuelArrivalMatchTimeSeconds =
                currentMatchTimeSeconds - MatchTrackerConstants.FUEL_FLIGHT_TIME_SECONDS;

        return isOurHubActiveAtMatchTime(fuelArrivalMatchTimeSeconds);
    }

    public static boolean isHubActiveNow() {
        return isOurHubActiveAtMatchTime(DriverStation.getMatchTime());
    }

    public static void forceGameData(char gameData) {
        final char upperCaseGameData = Character.toUpperCase(gameData);

        if (upperCaseGameData != MatchTrackerConstants.RED_ALLIANCE_GAME_DATA &&
                upperCaseGameData != MatchTrackerConstants.BLUE_ALLIANCE_GAME_DATA) {
            return;
        }

        forcedGameData = upperCaseGameData;
    }

    private static String getGameData() {
        if (forcedGameData != null)
            return String.valueOf(forcedGameData);

        return DriverStation.getGameSpecificMessage();
    }

    private static boolean isOurHubActiveAtMatchTime(double matchTimeSeconds) {
        if (DriverStation.isAutonomousEnabled())
            return true;

        if (!DriverStation.isTeleopEnabled())
            return false;

        final String gameData = getGameData();

        if (!isValidGameData(gameData)) {
            Logger.recordOutput("MatchTracker/HasValidGameData", false);

            /*
             * If game data is missing or invalid, default to true.
             * This prevents the robot from blocking shooting because of a Driver Station issue.
             */
            return true;
        }

        Logger.recordOutput("MatchTracker/HasValidGameData", true);

        final boolean isRedHubInactiveInShift1 =
                Character.toUpperCase(gameData.charAt(0)) == MatchTrackerConstants.RED_ALLIANCE_GAME_DATA;

        final boolean isOurHubInactiveInShift1 =
                isRedHubInactiveInShift1 == Flippable.isRedAlliance();

        final boolean isOurHubActiveInShift1 = !isOurHubInactiveInShift1;

        return calculateHubActiveDuringTeleopShift(matchTimeSeconds, isOurHubActiveInShift1);
    }

    private static boolean calculateHubActiveDuringTeleopShift(
            double matchTimeSeconds,
            boolean isOurHubActiveInShift1
    ) {
        if (matchTimeSeconds > MatchTrackerConstants.TRANSITION_SHIFT_START_TELEOP_TIME_SECONDS) {
            // Transition shift: both hubs are active.
            return true;
        }

        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_2_START_TELEOP_TIME_SECONDS) {
            // Shift 1.
            return isOurHubActiveInShift1;
        }

        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_3_START_TELEOP_TIME_SECONDS) {
            // Shift 2.
            return !isOurHubActiveInShift1;
        }

        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_4_START_TELEOP_TIME_SECONDS) {
            // Shift 3.
            return isOurHubActiveInShift1;
        }

        if (matchTimeSeconds > MatchTrackerConstants.ENDGAME_START_TELEOP_TIME_SECONDS) {
            // Shift 4.
            return !isOurHubActiveInShift1;
        }

        // Endgame: both hubs are active.
        return true;
    }

    private static boolean isValidGameData(String gameData) {
        if (gameData == null || gameData.isEmpty())
            return false;

        final char gameDataChar = Character.toUpperCase(gameData.charAt(0));

        return gameDataChar == MatchTrackerConstants.RED_ALLIANCE_GAME_DATA ||  gameDataChar == MatchTrackerConstants.BLUE_ALLIANCE_GAME_DATA;
    }

    public static void logMatchInfo() {
        final double matchTimeSeconds = DriverStation.getMatchTime();
        final double fuelArrivalMatchTimeSeconds =
                matchTimeSeconds - MatchTrackerConstants.FUEL_FLIGHT_TIME_SECONDS;

        Logger.recordOutput("MatchTimeSeconds", matchTimeSeconds);
        Logger.recordOutput("FuelArrivalMatchTimeSeconds", fuelArrivalMatchTimeSeconds);
        Logger.recordOutput("GameData", getGameData());
        Logger.recordOutput("IsGameDataForced", forcedGameData != null);
        Logger.recordOutput("IsRedAlliance", Flippable.isRedAlliance());
        Logger.recordOutput("IsHubActiveNow", isHubActiveNow());
        Logger.recordOutput("IsHubActiveForShooting", isHubActiveForShot());
    }
}