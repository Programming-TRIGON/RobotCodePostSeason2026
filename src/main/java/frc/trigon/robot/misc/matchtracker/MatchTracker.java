package frc.trigon.robot.misc.matchtracker;

import edu.wpi.first.wpilibj.DriverStation;
import frc.trigon.lib.utilities.flippable.Flippable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class MatchTracker {
    private static final LoggedNetworkBoolean HUB_ACTIVE_OVERRIDE = new LoggedNetworkBoolean("MatchTracker/HubActiveOverride", false);

    public static void logInfo() {
        Logger.recordOutput("MatchTracker/IsHubActive", isHubActive());
        Logger.recordOutput("MatchTracker/TimeUntilNextShift", getTimeUntilNextShift());
        Logger.recordOutput("MatchTracker/SecondsLeftInMatch", getCurrentMatchTimeSeconds());
    }

    public static boolean isHubActive() {
        final boolean isRedAlliance = Flippable.isRedAlliance();
        final double currentMatchTimeSeconds = getCurrentMatchTimeSeconds();
        final char autoWinner = getAutoWinner();
        final boolean isRedWonAuto = autoWinner == 'R';

        if (HUB_ACTIVE_OVERRIDE.get())
            return true;

        if (DriverStation.isAutonomousEnabled())
            return true;

        if (autoWinner != 'R' && autoWinner != 'B')
            return false;

        if (didShiftPass(MatchTrackerConstants.END_GAME_SHIFT_START_TIME_SECONDS))
            return true;

        if (didShiftPass(MatchTrackerConstants.FOURTH_SHIFT_START_TIME_SECONDS))
            return isRedAlliance == isRedWonAuto;

        if (didShiftPass(MatchTrackerConstants.THIRD_SHIFT_START_TIME_SECONDS))
            return isRedAlliance != isRedWonAuto;

        if (didShiftPass(MatchTrackerConstants.SECOND_SHIFT_START_TIME_SECONDS))
            return isRedAlliance == isRedWonAuto;

        if (didShiftPass(MatchTrackerConstants.FIRST_SHIFT_START_TIME_SECONDS))
            return isRedAlliance != isRedWonAuto;

        return currentMatchTimeSeconds < MatchTrackerConstants.TRANSITION_SHIFT_START_TIME_SECONDS;
    }

    private static boolean didShiftPass(double shiftStartTimeSeconds) {
        final double currentMatchTimeSeconds = getCurrentMatchTimeSeconds();
        return currentMatchTimeSeconds < shiftStartTimeSeconds + MatchTrackerConstants.HUB_ACTIVATION_EARLY_SECONDS;
    }

    private static double getTimeUntilNextShift() {
        final double currentMatchTimeSeconds = getCurrentMatchTimeSeconds();

        if (DriverStation.isAutonomousEnabled())
            return currentMatchTimeSeconds;

        if (!DriverStation.isTeleopEnabled())
            return 0;

        for (double shiftStartTime : MatchTrackerConstants.SHIFT_START_TIMES) {
            if (currentMatchTimeSeconds > shiftStartTime)
                return currentMatchTimeSeconds - shiftStartTime;
        }

        return currentMatchTimeSeconds;
    }

    private static double getCurrentMatchTimeSeconds() {
        return DriverStation.getMatchTime();
    }

    private static char getAutoWinner() {
        final String gameData = DriverStation.getGameSpecificMessage();
        final char autoWinner = gameData.charAt(0);

        if (gameData.isEmpty())
            return 0;

        return autoWinner == 'R' || autoWinner == 'B' ? autoWinner : 0;
    }
}
