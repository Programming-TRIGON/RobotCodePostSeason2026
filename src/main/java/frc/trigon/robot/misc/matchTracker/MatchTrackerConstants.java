package frc.trigon.robot.misc.matchTracker;

public class MatchTrackerConstants {
    public static final double
            TRANSITION_SHIFT_START_TIME_SECONDS = 130,
            SHIFT_2_START_TIME_SECONDS = 105,
            SHIFT_3_START_TIME_SECONDS = 80,
            SHIFT_4_START_TIME_SECONDS = 55,
            ENDGAME_START_TIME_SECONDS = 30;

    public static boolean WAS_HUB_ACTIVE_FOR_SHOOTING = false;

    /**
     * We allow shooting slightly before our hub becomes active,
     * because the fuel takes time to reach the hub.
     */
    public static final double
            FUEL_FLIGHT_TIME_SECONDS = 0.5,
            ALLIANCE_CHANGE_RUMBLE_TIME_SECONDS = 0.5;

    public static final char
            RED_ALLIANCE_GAME_DATA = 'R',
            BLUE_ALLIANCE_GAME_DATA = 'B';
}