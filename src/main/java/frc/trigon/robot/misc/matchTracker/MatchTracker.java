package frc.trigon.robot.misc.matchTracker;

import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.lib.utilities.flippable.Flippable;
import org.littletonrobotics.junction.Logger;

public class MatchTracker {
    private static boolean overrideGameData = false;

    public static boolean isHubActive() {
        return isOurHubActiveAtMatchTime(DriverStation.getMatchTime());
    }

    public static void overrideGameData() {
        overrideGameData = true;
    }

    public static void disableOverrideGameData() {
        overrideGameData = false;
    }

    private static String getGameData() {
        return DriverStation.getGameSpecificMessage();
    }

    private static boolean isOurHubActiveAtMatchTime(double matchTimeSeconds) {
        if (overrideGameData)
            return true;

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
        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_1_START_TELEOP_TIME_SECONDS) {
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

        return gameDataChar == MatchTrackerConstants.RED_ALLIANCE_GAME_DATA ||
                gameDataChar == MatchTrackerConstants.BLUE_ALLIANCE_GAME_DATA;
    }

    public static void logMatchInfo() {
        final double matchTimeSeconds = DriverStation.getMatchTime();

        Logger.recordOutput("MatchTimeSeconds", matchTimeSeconds);
        Logger.recordOutput("GameData", getGameData());
        Logger.recordOutput("IsHubAlwaysActiveOverridden", overrideGameData);
        Logger.recordOutput("IsRedAlliance", Flippable.isRedAlliance());
        Logger.recordOutput("IsHubActiveNow", isHubActive());
    }
}