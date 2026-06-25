package frc.trigon.robot.misc.matchTracker;

import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.commands.commandfactories.ShootingCommands;
import org.littletonrobotics.junction.Logger;

public class MatchTracker {
    private static boolean IS_HUB_STATE_ACTIVE = true;
    private static String SHIFT_TYPE = "autonomous";

    public static boolean hasHubActiveStateChanged() {
        final boolean currentHubActiveState = isHubActive();

        if (currentHubActiveState == IS_HUB_STATE_ACTIVE)
            return false;

        IS_HUB_STATE_ACTIVE = currentHubActiveState;
        return true;
    }

    public static boolean isHubActive() {
        if (ShootingCommands.SHOULD_OVERRIDE_GAME_DATA)
            return true;

        return isOurHubActiveAtMatchTime(DriverStation.getMatchTime());
    }

    public static boolean isOurHubActiveAtMatchTime(double matchTimeSeconds) {
        if (DriverStation.isAutonomousEnabled())
            return true;

        final String gameData = DriverStation.getGameSpecificMessage();

        if (!isValidGameData(gameData)) {
            Logger.recordOutput("MatchTracker/HasValidGameData", false);
            return true;
        }

        Logger.recordOutput("MatchTracker/HasValidGameData", true);

        final boolean isRedHubInactiveInShift1 =
                Character.toUpperCase(gameData.charAt(0)) == MatchTrackerConstants.RED_ALLIANCE_GAME_DATA;

        final boolean isOurHubInactiveInShift1 =
                isRedHubInactiveInShift1 == Flippable.isRedAlliance();

        final boolean isOurHubActiveInShift1 = !isOurHubInactiveInShift1;

        return shouldHubBeActiveDuringTeleopShift(matchTimeSeconds, isOurHubActiveInShift1);
    }

    public static boolean shouldHubBeActiveDuringTeleopShift(double matchTimeSeconds, boolean isOurHubActiveInShift1) {
        if (matchTimeSeconds > MatchTrackerConstants.AUTONOMOUS_TIME_SECOND) {
            SHIFT_TYPE = "autonomous";
            return true;
        }
        if (matchTimeSeconds > MatchTrackerConstants.TRANSITION_SHIFT_TIME_SECOND && matchTimeSeconds < MatchTrackerConstants.AUTONOMOUS_TIME_SECOND) {
            SHIFT_TYPE = "transition";
            return true;
        }
        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_1_START_TELEOP_TIME_SECONDS && matchTimeSeconds <  MatchTrackerConstants.TRANSITION_SHIFT_TIME_SECOND) {
            SHIFT_TYPE = "shift 1";
            return isOurHubActiveInShift1;
        }
        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_2_START_TELEOP_TIME_SECONDS && matchTimeSeconds < MatchTrackerConstants.SHIFT_1_START_TELEOP_TIME_SECONDS) {
            SHIFT_TYPE = "shift 2";
            return !isOurHubActiveInShift1;
        }
        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_3_START_TELEOP_TIME_SECONDS && matchTimeSeconds < MatchTrackerConstants.SHIFT_2_START_TELEOP_TIME_SECONDS) {
            SHIFT_TYPE = "shift 3";
            return isOurHubActiveInShift1;
        }
        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_4_START_TELEOP_TIME_SECONDS && matchTimeSeconds < MatchTrackerConstants.SHIFT_3_START_TELEOP_TIME_SECONDS) {
            SHIFT_TYPE = "shift 4";
            return !isOurHubActiveInShift1;
        }
        if (matchTimeSeconds < MatchTrackerConstants.SHIFT_4_START_TELEOP_TIME_SECONDS) {
            SHIFT_TYPE = "endgame";
            return isOurHubActiveInShift1;
        }

        return true;
    }

    public static boolean isValidGameData(String gameData) {
        if (gameData == null || gameData.isEmpty())
            return false;

        final char gameDataChar = Character.toUpperCase(gameData.charAt(0));

        return gameDataChar == MatchTrackerConstants.RED_ALLIANCE_GAME_DATA || gameDataChar == MatchTrackerConstants.BLUE_ALLIANCE_GAME_DATA;
    }

    private static int getShiftNumber(double matchTimeSeconds) {
        if (matchTimeSeconds > MatchTrackerConstants.AUTONOMOUS_TIME_SECOND)
            return 0;

        if (matchTimeSeconds > MatchTrackerConstants.TRANSITION_SHIFT_TIME_SECOND)
            return 1;

        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_1_START_TELEOP_TIME_SECONDS)
            return 2;

        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_2_START_TELEOP_TIME_SECONDS)
            return 3;

        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_3_START_TELEOP_TIME_SECONDS)
            return 4;

        if (matchTimeSeconds > MatchTrackerConstants.SHIFT_4_START_TELEOP_TIME_SECONDS)
            return 5;

        return 6;
    }

    private static double getNextShiftTimeSeconds(double matchTimeSeconds) {
        final int currentShiftNumber = getShiftNumber(matchTimeSeconds);

        return switch (currentShiftNumber) {
            case 0 -> MatchTrackerConstants.AUTONOMOUS_TIME_SECOND;
            case 1 -> MatchTrackerConstants.TRANSITION_SHIFT_TIME_SECOND;
            case 2 -> MatchTrackerConstants.SHIFT_1_START_TELEOP_TIME_SECONDS;
            case 3 -> MatchTrackerConstants.SHIFT_2_START_TELEOP_TIME_SECONDS;
            case 4 -> MatchTrackerConstants.SHIFT_3_START_TELEOP_TIME_SECONDS;
            case 5 -> MatchTrackerConstants.SHIFT_4_START_TELEOP_TIME_SECONDS;
            case 6 -> 0;
            default -> -1;
        };
    }

    public static double getTimeUntilShiftEndSeconds(double matchTimeSeconds) {
        return matchTimeSeconds - getNextShiftTimeSeconds(matchTimeSeconds);
    }

    public static void logMatchInfo() {
        final double matchTimeSeconds = DriverStation.getMatchTime();

        Logger.recordOutput("MatchTracker/MatchTimeSeconds", matchTimeSeconds);
        Logger.recordOutput("MatchTracker/IsHubActive", isHubActive());
        Logger.recordOutput("MatchTracker/TimeUntilAllianceShiftSeconds", getTimeUntilShiftEndSeconds(matchTimeSeconds));
        Logger.recordOutput("MatchTracker/ShiftType", SHIFT_TYPE);
    }
}