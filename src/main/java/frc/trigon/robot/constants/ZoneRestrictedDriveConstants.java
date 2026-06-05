package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions.zonerestrictions.ContainmentZone;

public class ZoneRestrictedDriveConstants {
    public static final BoundingBox ROBOT_BOUNDING_BOX = new BoundingBox(new Pose2d(), 1, 1);
    private static final double
            FIELD_BOUNDARY_MINIMUM_DISTANCE_METERS = 0.1,
            FIELD_BOUNDARY_BRAKING_ZONE_DISTANCE_METERS = 0.3;
    public static final ContainmentZone FIELD_BOUNDARY_ZONE = new ContainmentZone(
            FieldConstants.FIELD_BOUNDING_BOX,
            FIELD_BOUNDARY_MINIMUM_DISTANCE_METERS,
            FIELD_BOUNDARY_BRAKING_ZONE_DISTANCE_METERS
    );
}
