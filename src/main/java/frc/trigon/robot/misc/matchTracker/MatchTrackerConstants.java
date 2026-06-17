package frc.trigon.robot.misc.matchTracker;

public class MatchTrackerConstants {
    public static final double
            SHIFT_1_START_TELEOP_TIME_SECONDS = 130,
            SHIFT_2_START_TELEOP_TIME_SECONDS = 105,
            SHIFT_3_START_TELEOP_TIME_SECONDS = 80,
            SHIFT_4_START_TELEOP_TIME_SECONDS = 55,
            ENDGAME_START_TELEOP_TIME_SECONDS = 30;
    /**
     * We allow shooting slightly before our hub becomes active,
     * because the fuel takes time to reach the hub.
     */
    public static final double
            FUEL_FLIGHT_TIME_SECONDS = 0.5,
            RUMBLE_TIME_IN_SECONDS = 0.5,
            RUMBLE_POWER = 1;

    public static final char
            RED_ALLIANCE_GAME_DATA = 'R',
            BLUE_ALLIANCE_GAME_DATA = 'B';
}