package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.lib.utilities.zonerestricteddrive.ContainmentZone;
import frc.trigon.lib.utilities.zonerestricteddrive.ZoneRestriction;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;

public class ZoneRestrictedDriveCommand extends DriveRestrictedCommand {
    private static final double
            ROBOT_X_WIDTH_METERS = 1,
            ROBOT_Y_WIDTH_METERS = 1;
    private static final double
            FIELD_BOUNDARY_MINIMUM_DISTANCE_METERS = 0.1,
            FIELD_BOUNDARY_BRAKING_ZONE_DISTANCE_METERS = 0.3;
    private static final ContainmentZone FIELD_BOUNDARY_ZONE = new ContainmentZone(
            FieldConstants.FIELD_BOUNDING_BOX,
            FIELD_BOUNDARY_MINIMUM_DISTANCE_METERS,
            FIELD_BOUNDARY_BRAKING_ZONE_DISTANCE_METERS
    );

    private final ZoneRestriction[] zoneRestrictions;

    public ZoneRestrictedDriveCommand(boolean shouldRestrictToField, ZoneRestriction... zoneRestrictions) {
        super(DriveFrame.FIELD_RELATIVE);
        this.zoneRestrictions = shouldRestrictToField
                ? prependFieldBoundary(zoneRestrictions)
                : zoneRestrictions;

        if (RobotHardwareStats.isSimulation())
            logAllZoneBoundaries();
    }

    @Override
    protected void restrict(double shapedX, double shapedY, double shapedTheta) {
        final BoundingBox robotBox = getRobotBoundingBox();
        Translation2d translation = new Translation2d(shapedX, shapedY).unaryMinus();
        for (ZoneRestriction zone : zoneRestrictions)
            translation = zone.applyRestriction(translation, robotBox);
        translation = translation.unaryMinus();

        setRestrictedOutput(translation.getX(), translation.getY(), shapedTheta);
    }

    private static ZoneRestriction[] prependFieldBoundary(ZoneRestriction[] zones) {
        final ZoneRestriction[] all = new ZoneRestriction[zones.length + 1];
        all[0] = FIELD_BOUNDARY_ZONE;
        System.arraycopy(zones, 0, all, 1, zones.length);
        return all;
    }

    private void logAllZoneBoundaries() {
        for (int i = 0; i < zoneRestrictions.length; i++)
            zoneRestrictions[i].getBoundingBox().log("ZoneRestrictions/Zone" + i);
    }

    private BoundingBox getRobotBoundingBox() {
        final Pose2d robotPose = RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
        return new BoundingBox(robotPose, ROBOT_X_WIDTH_METERS, ROBOT_Y_WIDTH_METERS);
    }
}