package frc.trigon.robot.misc.matchTracker;

public class MatchTrackerConstants {
    public static final char
            RED_ALLIANCE_GAME_DATA = 'R',
            BLUE_ALLIANCE_GAME_DATA = 'B';

    static final double
            AUTONOMOUS_TIME_SECOND = 140,
            TRANSITION_SHIFT_TIME_SECOND = 130,
            SHIFT_1_START_TELEOP_TIME_SECONDS = 105,
            SHIFT_2_START_TELEOP_TIME_SECONDS = 80,
            SHIFT_3_START_TELEOP_TIME_SECONDS = 55,
            SHIFT_4_START_TELEOP_TIME_SECONDS = 30;
    /**
     * We allow shooting slightly before our hub becomes active,
     * because the fuel takes time to reach the hub.
     */
    static final double
            RUMBLE_TIME_SECONDS = 0.5,
            RUMBLE_POWER = 1;
}