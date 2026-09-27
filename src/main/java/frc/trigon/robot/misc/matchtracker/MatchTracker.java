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
        final char autoWinner = getAutoWinner();

        if (HUB_ACTIVE_OVERRIDE.get() || DriverStation.isAutonomousEnabled() || autoWinner == 0 || didShiftPassIncludingEarlyHubActivation(MatchTrackerConstants.END_GAME_START_TIME_SECONDS))
            return true;

        for (int shift = 1; shift <= 4; shift++) {
            final double shiftStartTime = getShiftStartTime(shift);

            if (didShiftPass(shiftStartTime)) {
                if (isAutoWinnerHubActive(shift) == didOurAllianceWinAuto(Flippable.isRedAlliance(), autoWinner))
                    return true;

                return didShiftPassIncludingEarlyHubActivation( shiftStartTime - MatchTrackerConstants.SHIFT_TIME_SECONDS);
            }
        }

        return true;
    }

    private static double getTimeUntilNextShift() {
        final double currentMatchTimeSeconds = getCurrentMatchTimeSeconds();

        if (DriverStation.isAutonomousEnabled() || didShiftPass(MatchTrackerConstants.END_GAME_START_TIME_SECONDS))
            return currentMatchTimeSeconds;

        if (currentMatchTimeSeconds < MatchTrackerConstants.TRANSITION_SHIFT_START_TIME_SECONDS)
            return currentMatchTimeSeconds - MatchTrackerConstants.FIRST_SHIFT_START_TIME_SECONDS;

        for (int shift = 1; shift <= 4; shift++) {
            final double shiftStartTime = getShiftStartTime(shift);

            if (currentMatchTimeSeconds > shiftStartTime)
                return currentMatchTimeSeconds - shiftStartTime;
        }

        return currentMatchTimeSeconds;
    }

    private static char getAutoWinner() {
        final String gameData = DriverStation.getGameSpecificMessage();

        if (gameData.isEmpty()) {
            return 0;
        }

        final char autoWinner = gameData.charAt(0);

        return autoWinner == 'R' || autoWinner == 'B' ? autoWinner : 0;
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

    private static double getShiftStartTime(int shift) {
        return MatchTrackerConstants.END_GAME_START_TIME_SECONDS + (5 - shift) * MatchTrackerConstants.SHIFT_TIME_SECONDS;
    }

    private static boolean isAutoWinnerHubActive(int shift) {
        return shift % 2 == 0;
    }

    private static double getCurrentMatchTimeSeconds() {
        return DriverStation.getMatchTime();
    }
}