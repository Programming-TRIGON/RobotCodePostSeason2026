package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.lib.hardware.RobotHardwareStats;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.lib.utilities.zonerestricteddrive.ZoneRestriction;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.ZoneRestrictedDriveConstants;

/**
 * A command that drives the robot while restricting movement relative to defined zones on the field.
 * Restricted zones slow and block movement into them.
 * Containment zones slow and block movement out of them.
 * All zone restrictions are applied sequentially, each further restricting the previous result.
 * Movement parallel to or away from a zone boundary is never restricted.
 */
public class ZoneRestrictedDriveCommand extends DriveRestrictedCommand {
    private final ZoneRestriction[] zoneRestrictions;

    /**
     * Creates a new ZoneRestrictedDriveCommand.
     *
     * @param shouldRestrictToField whether to restrict the robot from leaving the field boundary
     * @param zoneRestrictions      the zones to restrict movement relative to
     */
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
        all[0] = ZoneRestrictedDriveConstants.FIELD_BOUNDARY_ZONE;
        System.arraycopy(zones, 0, all, 1, zones.length);
        return all;
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