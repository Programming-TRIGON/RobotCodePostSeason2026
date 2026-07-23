package frc.trigon.robot.misc.matchTracker;

import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;
import org.littletonrobotics.junction.Logger;

public class MatchTracker {
    private static boolean IS_HUB_STATE_ACTIVE = true;
    private static boolean IS_HUB_ACTIVE_DURING_EVEN_SHIFTS = true;
    private static String ACTIVE_SHIFT_NAME = "Auton";

    public static boolean hasHubActiveStateChanged() {
        final boolean currentHubActiveState = isHubActive();

        if (currentHubActiveState == IS_HUB_STATE_ACTIVE)
            return false;

        IS_HUB_STATE_ACTIVE = currentHubActiveState;
        return true;
    }

    public static boolean isHubActive() {
        if (ShootingCommands.SHOULD_OVERRIDE_GAME_DATA != null && ShootingCommands.SHOULD_OVERRIDE_GAME_DATA.getAsBoolean())
            return true;

        return isOurHubActiveAtMatchTime(DriverStation.getMatchTime());
    }

    public static boolean isOurHubActiveAtMatchTime(double matchTimeSeconds) {
        if (DriverStation.isAutonomousEnabled()) {
            ACTIVE_SHIFT_NAME = "Auton";
            return true;
        }
        if (matchTimeSeconds <= 30) {
            ACTIVE_SHIFT_NAME = "Endgame";
            return true;
        }

        final String gameData = DriverStation.getGameSpecificMessage();

        if (!isGameDataValid(gameData)) {
            Logger.recordOutput("MatchTracker/HasValidGameData", false);
            return true;
        }

        Logger.recordOutput("MatchTracker/HasValidGameData", true);

        final boolean isRedHubInactiveInShift1 =
                Character.toUpperCase(gameData.charAt(0)) == MatchTrackerConstants.RED_ALLIANCE_GAME_DATA;

        IS_HUB_ACTIVE_DURING_EVEN_SHIFTS =
                isRedHubInactiveInShift1 == Flippable.isRedAlliance();

        final boolean isOurHubActiveInShift1 = !IS_HUB_ACTIVE_DURING_EVEN_SHIFTS;

        return shouldHubBeActiveDuringTeleopShift(matchTimeSeconds, isOurHubActiveInShift1);
    }

    public static boolean shouldHubBeActiveDuringTeleopShift(double matchTimeSeconds, boolean isOurHubActiveInShift1) {
        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_1_START_TIME_SECONDS) {
            ACTIVE_SHIFT_NAME = "Transition";
            return true;
        }
        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_2_START_TIME_SECONDS) {
            ACTIVE_SHIFT_NAME = "Shift 1";
            return isOurHubActiveInShift1;
        }
        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_3_START_TIME_SECONDS) {
            ACTIVE_SHIFT_NAME = "Shift 2";
            return !isOurHubActiveInShift1;
        }
        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_4_START_TIME_SECONDS) {
            ACTIVE_SHIFT_NAME = "Shift 3";
            return isOurHubActiveInShift1;
        }
        if (matchTimeSeconds > MatchTrackerConstants.ENDGAME_START_TIME_SECONDS) {
            ACTIVE_SHIFT_NAME = "Shift 4";
            return !isOurHubActiveInShift1;
        }
        ACTIVE_SHIFT_NAME = "Error";
        return true;
    }

    public static boolean isGameDataValid(String gameData) {
        if (gameData == null || gameData.isEmpty())
            return false;

        final char gameDataChar = Character.toUpperCase(gameData.charAt(0));

        return gameDataChar == MatchTrackerConstants.RED_ALLIANCE_GAME_DATA || gameDataChar == MatchTrackerConstants.BLUE_ALLIANCE_GAME_DATA;
    }

    private static int getShiftNumber(double matchTimeSeconds) {
        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_1_START_TIME_SECONDS)
            return 0;
        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_2_START_TIME_SECONDS)
            return 1;
        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_3_START_TIME_SECONDS)
            return 2;
        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_4_START_TIME_SECONDS)
            return 3;
        if (matchTimeSeconds > MatchTrackerConstants.ENDGAME_START_TIME_SECONDS)
            return 4;
        return 5;
    }

    public static double getTimeUntilHubDeactivatesSeconds() {
        final double currentMatchTimeSeconds = DriverStation.getMatchTime();

        final double timeUntilNextShiftSeconds = getTimeUntilShiftEndSeconds(currentMatchTimeSeconds);
        final int currentShiftNumber = getShiftNumber(currentMatchTimeSeconds);

        if (currentShiftNumber >= 4)
            return currentMatchTimeSeconds;

        return currentShiftNumber % 2 == 0 ^ IS_HUB_ACTIVE_DURING_EVEN_SHIFTS ?
                currentShiftNumber == 3 ? currentMatchTimeSeconds : timeUntilNextShiftSeconds + MatchTrackerConstants.SHIFT_DURATION_SECONDS :
                timeUntilNextShiftSeconds;
    }

    public static double getTimeUntilHubActivatesSeconds() {
        final double currentMatchTimeSeconds = DriverStation.getMatchTime();

        final double timeUntilNextShiftSeconds = getTimeUntilShiftEndSeconds(currentMatchTimeSeconds);
        final int currentShiftNumber = getShiftNumber(currentMatchTimeSeconds);

        if (currentShiftNumber >= 4)
            return -1;

        return currentShiftNumber % 2 == 0 ^ IS_HUB_ACTIVE_DURING_EVEN_SHIFTS ?
                timeUntilNextShiftSeconds :
                currentShiftNumber == 3 ? -1 : timeUntilNextShiftSeconds + MatchTrackerConstants.SHIFT_DURATION_SECONDS;
    }

    public static double getTimeUntilShiftEndSeconds(double matchTimeSeconds) {
        return matchTimeSeconds - getNextShiftTimeSeconds(matchTimeSeconds);
    }

    private static double getNextShiftTimeSeconds(double matchTimeSeconds) {
        final int currentShiftNumber = getShiftNumber(matchTimeSeconds);

        return switch (currentShiftNumber) {
            case 0 -> MatchTrackerConstants.SHIFT_1_START_TIME_SECONDS;
            case 1 -> MatchTrackerConstants.SHIFT_2_START_TIME_SECONDS;
            case 2 -> MatchTrackerConstants.SHIFT_3_START_TIME_SECONDS;
            case 3 -> MatchTrackerConstants.SHIFT_4_START_TIME_SECONDS;
            case 4 -> MatchTrackerConstants.ENDGAME_START_TIME_SECONDS;
            default -> 0;
        };
    }

    public static void logMatchInfo() {
        final double matchTimeSeconds = DriverStation.getMatchTime();

        Logger.recordOutput("MatchTracker/MatchTimeSeconds", matchTimeSeconds);
        Logger.recordOutput("MatchTracker/IsHubActive", isHubActive());
        Logger.recordOutput("MatchTracker/TimeUntilAllianceShiftSeconds", getTimeUntilShiftEndSeconds(matchTimeSeconds));
        Logger.recordOutput("MatchTracker/CurrentShiftName", ACTIVE_SHIFT_NAME);
    }
}