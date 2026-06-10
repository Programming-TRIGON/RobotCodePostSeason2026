package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.zonerestrictions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions.DriveRestriction;
import frc.trigon.robot.constants.ZoneRestrictedDriveConstants;

/**
 * Restricts the robot's movement based on defined zones on the field.
 * All zone restrictions are applied sequentially, each further restricting the previous result.
 * Useful for keeping the robot inside the field boundary or out of specific regions such as opponent protected areas.
 */
public class ZoneRestrictedDrive implements DriveRestriction {
    private final ZoneRestriction[] zoneRestrictions;

    /**
     * Creates a new zone restriction.
     *
     * @param shouldRestrictToField whether to restrict the robot from leaving the field boundary
     * @param zoneRestrictions      the zones to restrict movement relative to
     */
    public ZoneRestrictedDrive(boolean shouldRestrictToField, ZoneRestriction... zoneRestrictions) {
        this.zoneRestrictions = shouldRestrictToField
                ? getZoneRestrictionsWithFieldRestriction(zoneRestrictions)
                : zoneRestrictions;
        logAllZoneBoundaries();
    }

    @Override
    public Translation2d applyRestrictionToTranslation(Translation2d targetTranslation) {
        final BoundingBox robotBox = getRobotBoundingBox();
        Translation2d translation = targetTranslation;
        for (ZoneRestriction zone : zoneRestrictions)
            translation = zone.applyRestriction(translation, robotBox);
        return translation;
    }

    private static ZoneRestriction[] getZoneRestrictionsWithFieldRestriction(ZoneRestriction[] zones) {
        final ZoneRestriction[] allZones = new ZoneRestriction[zones.length + 1];

        allZones[0] = ZoneRestrictedDriveConstants.FIELD_BOUNDARY_ZONE;
        System.arraycopy(zones, 0, allZones, 1, zones.length);

        return allZones;
    }

    private void logAllZoneBoundaries() {
        for (int i = 0; i < zoneRestrictions.length; i++)
            zoneRestrictions[i].getBoundingBox().log("ZoneRestrictions/Zone" + i);
    }

    private BoundingBox getRobotBoundingBox() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        final BoundingBox robotBoundingBox = ZoneRestrictedDriveConstants.ROBOT_RELATIVE_BOUNDING_BOX;

        final Translation2d robotCenterFieldRelative = robotBoundingBox.getCenter().getTranslation()
                .rotateBy(robotPose.getRotation())
                .plus(robotPose.getTranslation());

        return new BoundingBox(
                new Pose2d(robotCenterFieldRelative, robotPose.getRotation()),
                robotBoundingBox.getXWidth(),
                robotBoundingBox.getYWidth()
        );
    }
}