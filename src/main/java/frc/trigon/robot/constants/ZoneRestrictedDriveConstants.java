package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.zonerestrictions.ContainmentZone;

public class ZoneRestrictedDriveConstants {
    public static final BoundingBox ROBOT_RELATIVE_BOUNDING_BOX = new BoundingBox(new Translation2d(), new Translation2d());
    private static final double
            FIELD_BOUNDARY_MINIMUM_DISTANCE_METERS = 0.1,
            FIELD_BOUNDARY_BRAKING_ZONE_DISTANCE_METERS = 0.3;
    public static final ContainmentZone FIELD_BOUNDARY_ZONE = new ContainmentZone(
            FieldConstants.FIELD_BOUNDING_BOX,
            FIELD_BOUNDARY_MINIMUM_DISTANCE_METERS,
            FIELD_BOUNDARY_BRAKING_ZONE_DISTANCE_METERS
    );
}
