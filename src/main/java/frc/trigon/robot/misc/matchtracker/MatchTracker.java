package frc.trigon.robot.misc.matchtracker;

import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.lib.utilities.flippable.Flippable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class MatchTracker {
    private static final LoggedNetworkBoolean HUB_ACTIVE_OVERRIDE = new LoggedNetworkBoolean("SmartDashboard/MatchTracker/HubActiveOverride", false);

    public static void logInfo() {
        Logger.recordOutput("MatchTracker/IsHubActive", isHubActive());
        Logger.recordOutput("MatchTracker/TimeUntilNextShift", getTimeUntilNextShift());
        Logger.recordOutput("MatchTracker/SecondsLeftInMatch", getCurrentMatchTimeSeconds());
    }

    public static boolean isHubActive() {
        if (HUB_ACTIVE_OVERRIDE.get() || DriverStation.isAutonomousEnabled())
            return true;

        final boolean isRedAlliance = Flippable.isRedAlliance();
        final double currentMatchTimeSeconds = getCurrentMatchTimeSeconds();
        final char autoWinner = getAutoWinner();
        final boolean didOurAllianceWinAuto = didOurAllianceWinAuto(isRedAlliance, autoWinner);

        if (autoWinner == 0 || didShiftPassIncludingEarlyHubActivation(MatchTrackerConstants.END_GAME_START_TIME_SECONDS))
            return true;

        if (currentMatchTimeSeconds < MatchTrackerConstants.FOURTH_SHIFT_START_TIME_SECONDS)
            return didOurAllianceWinAuto;
//      No need to check early activation for the next shift, since it is endgame and is already handled separately.

        if (currentMatchTimeSeconds < MatchTrackerConstants.THIRD_SHIFT_START_TIME_SECONDS) {
            if (!didOurAllianceWinAuto)
                return true;

            return didShiftPassIncludingEarlyHubActivation(MatchTrackerConstants.FOURTH_SHIFT_START_TIME_SECONDS);
        }

        if (currentMatchTimeSeconds < MatchTrackerConstants.SECOND_SHIFT_START_TIME_SECONDS) {
            if (didOurAllianceWinAuto)
                return true;

            return didShiftPassIncludingEarlyHubActivation(MatchTrackerConstants.THIRD_SHIFT_START_TIME_SECONDS);
        }

        if (currentMatchTimeSeconds < MatchTrackerConstants.FIRST_SHIFT_START_TIME_SECONDS) {
            if (!didOurAllianceWinAuto)
                return true;

            return didShiftPassIncludingEarlyHubActivation(MatchTrackerConstants.SECOND_SHIFT_START_TIME_SECONDS);
        }

        return true;
    }

    private static double getTimeUntilNextShift() {
        final double currentMatchTimeSeconds = getCurrentMatchTimeSeconds();

        if (DriverStation.isAutonomousEnabled() || didShiftPass(MatchTrackerConstants.END_GAME_START_TIME_SECONDS))
            return currentMatchTimeSeconds;

        if (didShiftPass(MatchTrackerConstants.FOURTH_SHIFT_START_TIME_SECONDS))
            return currentMatchTimeSeconds - MatchTrackerConstants.END_GAME_START_TIME_SECONDS;

        if (didShiftPass(MatchTrackerConstants.THIRD_SHIFT_START_TIME_SECONDS))
            return currentMatchTimeSeconds - MatchTrackerConstants.FOURTH_SHIFT_START_TIME_SECONDS;

        if (didShiftPass(MatchTrackerConstants.SECOND_SHIFT_START_TIME_SECONDS))
            return currentMatchTimeSeconds - MatchTrackerConstants.THIRD_SHIFT_START_TIME_SECONDS;

        if (didShiftPass(MatchTrackerConstants.FIRST_SHIFT_START_TIME_SECONDS))
            return currentMatchTimeSeconds - MatchTrackerConstants.SECOND_SHIFT_START_TIME_SECONDS;

        if (currentMatchTimeSeconds < MatchTrackerConstants.TRANSITION_SHIFT_START_TIME_SECONDS)
            return currentMatchTimeSeconds - MatchTrackerConstants.FIRST_SHIFT_START_TIME_SECONDS;

        return currentMatchTimeSeconds;
    }

    private static boolean didShiftPassIncludingEarlyHubActivation(double shiftStartTimeSeconds) {
        final double currentMatchTimeSeconds = getCurrentMatchTimeSeconds();
        return currentMatchTimeSeconds < shiftStartTimeSeconds + MatchTrackerConstants.HUB_ACTIVATION_EARLY_SECONDS;
    }

    private static boolean didShiftPass(double shiftStartTimeSeconds) {
        final double currentMatchTimeSeconds = getCurrentMatchTimeSeconds();
        return currentMatchTimeSeconds < shiftStartTimeSeconds;
    }

    private static boolean didOurAllianceWinAuto(boolean isRedAlliance, char autoWinner) {
        return isRedAlliance == (autoWinner == 'R');
    }

    private static double getCurrentMatchTimeSeconds() {
        return DriverStation.getMatchTime();
    }

    private static char getAutoWinner() {
        final String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty())
            return 0;

        final char autoWinner = gameData.charAt(0);

        return autoWinner == 'R' || autoWinner == 'B' ? autoWinner : 0;
    }
}