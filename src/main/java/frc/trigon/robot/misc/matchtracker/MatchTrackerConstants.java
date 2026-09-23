package frc.trigon.robot.misc.matchtracker;

public class MatchTrackerConstants {
    static final double
            TRANSITION_SHIFT_START_TIME_SECONDS = 140,
            FIRST_SHIFT_START_TIME_SECONDS = 130,
            SECOND_SHIFT_START_TIME_SECONDS = 105,
            THIRD_SHIFT_START_TIME_SECONDS = 80,
            FOURTH_SHIFT_START_TIME_SECONDS = 55,
            END_GAME_SHIFT_START_TIME_SECONDS = 30,
            HUB_ACTIVATION_EARLY_SECONDS = 2;

    static final double[] SHIFT_START_TIMES = {
            MatchTrackerConstants.TRANSITION_SHIFT_START_TIME_SECONDS,
            MatchTrackerConstants.FIRST_SHIFT_START_TIME_SECONDS,
            MatchTrackerConstants.SECOND_SHIFT_START_TIME_SECONDS,
            MatchTrackerConstants.THIRD_SHIFT_START_TIME_SECONDS,
            MatchTrackerConstants.FOURTH_SHIFT_START_TIME_SECONDS,
            MatchTrackerConstants.END_GAME_SHIFT_START_TIME_SECONDS
    };
}
