package frc.trigon.robot.misc.matchTracker;

import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.lib.utilities.flippable.Flippable;
import org.littletonrobotics.junction.Logger;

public class MatchTracker {
    private static Character forcedGameData = null;

    public static boolean isHubActiveForShooting() {
        final double currentMatchTimeSeconds = DriverStation.getMatchTime();
        final double expectedFuelArrivalMatchTimeSeconds = currentMatchTimeSeconds - MatchTrackerConstants.FUEL_FLIGHT_TIME_SECONDS;

        return isOurHubActiveAtMatchTime(expectedFuelArrivalMatchTimeSeconds);
    }

    public static boolean isHubActiveNow() {
        return isOurHubActiveAtMatchTime(DriverStation.getMatchTime());
    }

    public static void forceGameData(char gameData) {
        final char gameDataUpperCase = Character.toUpperCase(gameData);

        if (gameDataUpperCase != MatchTrackerConstants.RED_ALLIANCE_GAME_DATA &&
                gameDataUpperCase != MatchTrackerConstants.BLUE_ALLIANCE_GAME_DATA) {
            return;
        }

        forcedGameData = gameDataUpperCase;
    }

    public static void clearForcedGameData() {
        forcedGameData = null;
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
            return true;
        }

        Logger.recordOutput("MatchTracker/HasValidGameData", true);

        final boolean isRedHubInactiveInShift1 = Character.toUpperCase(gameData.charAt(0)) == MatchTrackerConstants.RED_ALLIANCE_GAME_DATA;
        final boolean isOurHubInactiveInShift1 = isRedHubInactiveInShift1 == Flippable.isRedAlliance();
        final boolean isOurHubActiveInShift1 = !isOurHubInactiveInShift1;

        return calculateHubActiveDuringTeleopShift(matchTimeSeconds, isOurHubActiveInShift1);
    }

    private static boolean calculateHubActiveDuringTeleopShift(
            double matchTimeSeconds,
            boolean isOurHubActiveInShift1
    ) {
        if (matchTimeSeconds > MatchTrackerConstants.TRANSITION_SHIFT_START_TELEOP_TIME_SECONDS) {
            return true;
        }

        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_2_START_TELEOP_TIME_SECONDS) {
            return isOurHubActiveInShift1;
        }

        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_3_START_TELEOP_TIME_SECONDS) {
            return !isOurHubActiveInShift1;
        }

        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_4_START_TELEOP_TIME_SECONDS) {
            return isOurHubActiveInShift1;
        }

        if (matchTimeSeconds > MatchTrackerConstants.ENDGAME_START_TELEOP_TIME_SECONDS) {
            return !isOurHubActiveInShift1;
        }

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
        Logger.recordOutput("IsHubActiveForShooting", isHubActiveForShooting());
    }
}