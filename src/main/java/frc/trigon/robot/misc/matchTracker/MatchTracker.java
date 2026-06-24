package frc.trigon.robot.misc.matchTracker;

import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;
import org.littletonrobotics.junction.Logger;

public class MatchTracker {
    public static boolean isHubActive() {
        if (ShootingCommands.shouldOverrideGameData)
            return true;

        return isOurHubActiveAtMatchTime(DriverStation.getMatchTime());
    }

    public static boolean isOurHubActiveAtMatchTime(double matchTimeSeconds) {
        if (DriverStation.isAutonomousEnabled())
            return true;

        final String gameData = DriverStation.getGameSpecificMessage();

        if (!MatchTracker.isValidGameData(gameData)) {
            Logger.recordOutput("MatchTracker/HasValidGameData", false);
            return true;
        }

        Logger.recordOutput("MatchTracker/HasValidGameData", true);

        final boolean isRedHubInactiveInShift1 = Character.toUpperCase(gameData.charAt(0)) == MatchTrackerConstants.RED_ALLIANCE_GAME_DATA;
        final boolean isOurHubInactiveInShift1 = isRedHubInactiveInShift1 == Flippable.isRedAlliance();
        final boolean isOurHubActiveInShift1 = !isOurHubInactiveInShift1;

        return MatchTracker.calculateHubActiveDuringTeleopShift(matchTimeSeconds, isOurHubActiveInShift1);
    }

    public static boolean calculateHubActiveDuringTeleopShift(double matchTimeSeconds, boolean isOurHubActiveInShift1) {
        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_1_START_TELEOP_TIME_SECONDS)
            return true;

        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_2_START_TELEOP_TIME_SECONDS)
            return isOurHubActiveInShift1;

        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_3_START_TELEOP_TIME_SECONDS)
            return !isOurHubActiveInShift1;

        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_4_START_TELEOP_TIME_SECONDS)
            return isOurHubActiveInShift1;

        if (matchTimeSeconds > MatchTrackerConstants.ENDGAME_START_TELEOP_TIME_SECONDS)
            return !isOurHubActiveInShift1;

        return true;
    }

    public static boolean isValidGameData(String gameData) {
        if (gameData == null || gameData.isEmpty())
            return false;

        final char gameDataChar = Character.toUpperCase(gameData.charAt(0));

        return gameDataChar == MatchTrackerConstants.RED_ALLIANCE_GAME_DATA ||
                gameDataChar == MatchTrackerConstants.BLUE_ALLIANCE_GAME_DATA;
    }

    public static void logMatchInfo() {
        Logger.recordOutput("MatchTimeSeconds", DriverStation.getMatchTime());
        Logger.recordOutput("GameData", DriverStation.getGameSpecificMessage());
        Logger.recordOutput("IsRedAlliance", Flippable.isRedAlliance());
        Logger.recordOutput("IsHubActiveNow", isHubActive());
    }
}