package frc.trigon.robot.misc.matchtracker;

import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.lib.utilities.flippable.Flippable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class MatchTracker {
    private static final LoggedNetworkBoolean HUB_ACTIVE_OVERRIDE = new LoggedNetworkBoolean("SmartDashboard/MatchTracker/HubActiveOverride", false);

    public static void logInfo() {
        Logger.recordOutput("MatchTracker/IsHubActive", isHubActive());
        Logger.recordOutput("MatchTracker/SecondsUntilNextShift", getSecondsUntilNextShift());
        Logger.recordOutput("MatchTracker/SecondsLeftInMatch", getCurrentMatchTimeSeconds());
    }

    public static boolean isHubActive() {
        final char autoWinner = getAutoWinner();
        final double currentMatchTimeSeconds = getCurrentMatchTimeSeconds();
        final boolean isRedAlliance = Flippable.isRedAlliance();

        if (HUB_ACTIVE_OVERRIDE.get() || DriverStation.isAutonomousEnabled() || autoWinner == ' ' || didShiftPassIncludingEarlyHubActivation(MatchTrackerConstants.END_GAME_START_TIME_SECONDS))
            return true;

        for (int shift = 4; shift >= 1; shift--) {
            final double shiftStartTimeSeconds = getShiftStartTimeSeconds(shift);

            if (didShiftPass(currentMatchTimeSeconds, shiftStartTimeSeconds)) {
                if (isAutoWinnerHubActive(shift) == didOurAllianceWinAuto(isRedAlliance, autoWinner))
                    return true;

                return didShiftPassIncludingEarlyHubActivation(shiftStartTimeSeconds, shiftStartTimeSeconds - MatchTrackerConstants.SHIFT_TIME_SECONDS);
            }
        }

        return true;
    }

    private static double getSecondsUntilNextShift() {
        final double currentMatchTimeSeconds = getCurrentMatchTimeSeconds();

        if (DriverStation.isAutonomousEnabled() || didShiftPass(currentMatchTimeSeconds ,MatchTrackerConstants.END_GAME_START_TIME_SECONDS))
            return currentMatchTimeSeconds;

        for (int shift = 4; shift >= 1; shift--) {
            final double shiftStartTimeSeconds = getShiftStartTimeSeconds(shift);

            if (didShiftPass(currentMatchTimeSeconds, shiftStartTimeSeconds))
                return currentMatchTimeSeconds - (shiftStartTimeSeconds - MatchTrackerConstants.SHIFT_TIME_SECONDS);
        }

        if (didShiftPass(currentMatchTimeSeconds, MatchTrackerConstants.TRANSITION_SHIFT_START_TIME_SECONDS))
            return currentMatchTimeSeconds - MatchTrackerConstants.FIRST_SHIFT_START_TIME_SECONDS;

        return currentMatchTimeSeconds;
    }

    private static char getAutoWinner() {
        final String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty())
            return ' ';

        final char autoWinner = gameData.charAt(0);

        return autoWinner == 'R' || autoWinner == 'B' ? autoWinner : ' ';
    }

    private static boolean didShiftPassIncludingEarlyHubActivation(double currentMatchTimeSeconds, double shiftStartTimeSeconds) {
        return currentMatchTimeSeconds <= shiftStartTimeSeconds + MatchTrackerConstants.HUB_ACTIVATION_EARLY_SECONDS;
    }

    private static boolean didShiftPass(double currentMatchTimeSeconds, double shiftStartTimeSeconds) {
        return currentMatchTimeSeconds <= shiftStartTimeSeconds;
    }

    private static boolean didOurAllianceWinAuto(boolean isRedAlliance, char autoWinner) {
        return isRedAlliance == (autoWinner == 'R');
    }

    private static double getShiftStartTimeSeconds(int shift) {
        return MatchTrackerConstants.END_GAME_START_TIME_SECONDS + (5 - shift) * MatchTrackerConstants.SHIFT_TIME_SECONDS;
    }

    private static boolean isAutoWinnerHubActive(int shift) {
        return shift % 2 == 0;
    }

    private static double getCurrentMatchTimeSeconds() {
        return DriverStation.getMatchTime();
    }
}