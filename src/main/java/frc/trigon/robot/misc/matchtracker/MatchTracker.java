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
        if (HUB_ACTIVE_OVERRIDE.get())
            return true;

        if (DriverStation.isAutonomousEnabled())
            return true;

        final boolean isRedAlliance = Flippable.isRedAlliance();
        final double currentMatchTimeSeconds = getCurrentMatchTimeSeconds();
        final char autoWinner = getAutoWinner();

        if (autoWinner != 'R' && autoWinner != 'B') {
            return false;
        }

        final boolean didOurAllianceWinAuto = isRedAlliance == (autoWinner == 'R');

        if (didShiftPass(MatchTrackerConstants.END_GAME_SHIFT_START_TIME_SECONDS))
            return true;

        for (int shift = 4; shift >= 1; shift--) {
            final double shiftStartTime =
                    MatchTrackerConstants.END_GAME_SHIFT_START_TIME_SECONDS
                            + (5 - shift) * MatchTrackerConstants.SHIFT_TIME_SECONDS;

            if (didShiftPass(shiftStartTime)) {
                final boolean autoWinnerHubActive = shift % 2 == 0;

                final boolean hubActive = autoWinnerHubActive == didOurAllianceWinAuto;

                if (hubActive)
                    return true;

                final double nextShiftStartTime =
                        shift == 4
                                ? MatchTrackerConstants.END_GAME_SHIFT_START_TIME_SECONDS
                                : MatchTrackerConstants.END_GAME_SHIFT_START_TIME_SECONDS
                                + (4 - shift) * MatchTrackerConstants.SHIFT_TIME_SECONDS;

                return didShiftPass(nextShiftStartTime);
            }
        }
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

        if (currentMatchTimeSeconds > MatchTrackerConstants.TRANSITION_SHIFT_START_TIME_SECONDS)
            return currentMatchTimeSeconds - MatchTrackerConstants.TRANSITION_SHIFT_START_TIME_SECONDS;

        for (int i = 4; i >= 0; i--) {
            final double shiftStartTime =
                    MatchTrackerConstants.END_GAME_SHIFT_START_TIME_SECONDS
                            + i * MatchTrackerConstants.SHIFT_TIME_SECONDS;

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

        if (gameData.isEmpty()) {
            return 0;
        }

        final char autoWinner = gameData.charAt(0);

        return autoWinner == 'R' || autoWinner == 'B' ? autoWinner : 0;
    }
}
