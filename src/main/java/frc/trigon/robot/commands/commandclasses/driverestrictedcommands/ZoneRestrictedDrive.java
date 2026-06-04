package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.lib.utilities.zonerestricteddrive.ZoneRestriction;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.commands.DriveRestriction;
import frc.trigon.robot.constants.ZoneRestrictedDriveConstants;

public class ZoneRestrictedDrive implements DriveRestriction {
    private final ZoneRestriction[] zoneRestrictions;

    public ZoneRestrictedDrive(boolean shouldRestrictToField, ZoneRestriction... zoneRestrictions) {
        this.zoneRestrictions = shouldRestrictToField
                ? getZoneRestrictionsWithFieldRestriction(zoneRestrictions)
                : zoneRestrictions;

            logAllZoneBoundaries();
    }

    @Override
    public Translation2d applyRestrictionToTranslation(Translation2d targetTranslation) {
        final BoundingBox robotBox = getRobotBoundingBox();
        Translation2d translation = targetTranslation.unaryMinus();
        for (ZoneRestriction zone : zoneRestrictions)
            translation = zone.applyRestriction(translation, robotBox);
        return translation.unaryMinus();
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
        return new BoundingBox(robotPose, ZoneRestrictedDriveConstants.ROBOT_X_WIDTH_METERS, ZoneRestrictedDriveConstants.ROBOT_Y_WIDTH_METERS);
    }
}