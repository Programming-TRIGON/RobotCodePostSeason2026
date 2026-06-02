package frc.trigon.robot.constants;

import frc.trigon.lib.utilities.zonerestricteddrive.ContainmentZone;

public class ZoneRestrictedDriveConstants {
    public static final double
            ROBOT_X_WIDTH_METERS = 1,
            ROBOT_Y_WIDTH_METERS = 1;
    public static final double
            FIELD_BOUNDARY_MINIMUM_DISTANCE_METERS = 0.1,
            FIELD_BOUNDARY_BRAKING_ZONE_DISTANCE_METERS = 0.3;
    public static final ContainmentZone FIELD_BOUNDARY_ZONE = new ContainmentZone(
            FieldConstants.FIELD_BOUNDING_BOX,
            FIELD_BOUNDARY_MINIMUM_DISTANCE_METERS,
            FIELD_BOUNDARY_BRAKING_ZONE_DISTANCE_METERS
    );
}
