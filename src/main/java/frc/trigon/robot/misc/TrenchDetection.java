package frc.trigon.robot.misc;

import edu.wpi.first.math.geometry.*;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class TrenchDetection {
    private static final double
            HOOD_OFFSET_X_METERS = -150 / 1000.0,
            HOOD_OFFSET_Y_METERS = 484.6 / 1000.0,
            HOOD_WIDTH_METERS = 619 / 1000.0,
            HOOD_LENGTH_METERS = 256 / 1000.0,
            HOOD_BOUNDING_BOX_EXPANSION_METERS = 0.1;

    private static final double TRENCH_POSE_PREDICTION_TIME_SECONDS = 0.2;

    /**
     * Checks whether the hood is currently in, will be in, or is passing through a trench zone.
     * The pass-through checks catch the case where the robot moves fast enough to cross the entire
     * trench between two samples, so neither the current nor predicted position overlaps the trench.
     *
     * @return whether the hood is in or passing through a trench
     */
    public static boolean isHoodInTrenchZone() {
        return isHoodInTrench()
                || willHoodBeInTrench()
                || (isHoodBeforeTrench() && willHoodBeAfterTrench())
                || (isHoodAfterTrench() && willHoodBeBeforeTrench());
    }

    /**
     * Logs all trench bounding boxes and the current hood bounding box for visualization in AdvantageScope.
     */
    public static void logTrenchBoundingBoxes() {
        FieldConstants.BLUE_CLOSE_TRENCH_BOUNDING_BOX.log("Zones/BlueCloseTrench");
        FieldConstants.BLUE_FAR_TRENCH_BOUNDING_BOX.log("Zones/BlueFarTrench");
        FieldConstants.RED_CLOSE_TRENCH_BOUNDING_BOX.log("Zones/RedCloseTrench");
        FieldConstants.RED_FAR_TRENCH_BOUNDING_BOX.log("Zones/RedFarTrench");
        getHoodBoundingBox(getRobotPose()).log("Zones/HoodBoundingBox");
        getHoodBoundingBox(getPredictedRobotPose()).log("Zones/PredictedHoodBoundingBox");
        Logger.recordOutput("Zones/IsHoodInTrenchZone", isHoodInTrenchZone());
    }

    private static boolean isHoodInTrench() {
        return isHoodBoundingBoxInTrenchZone(getHoodBoundingBox(getRobotPose()));
    }

    private static boolean willHoodBeInTrench() {
        return isHoodBoundingBoxInTrenchZone(getHoodBoundingBox(getPredictedRobotPose()));
    }

    private static boolean isHoodBeforeTrench() {
        return isHoodCenterBeforeTrench(getHoodBoundingBox(getRobotPose()).getCenter().getTranslation());
    }

    private static boolean willHoodBeBeforeTrench() {
        return isHoodCenterBeforeTrench(getHoodBoundingBox(getPredictedRobotPose()).getCenter().getTranslation());
    }

    private static boolean isHoodAfterTrench() {
        return isHoodCenterAfterTrench(getHoodBoundingBox(getRobotPose()).getCenter().getTranslation());
    }

    private static boolean willHoodBeAfterTrench() {
        return isHoodCenterAfterTrench(getHoodBoundingBox(getPredictedRobotPose()).getCenter().getTranslation());
    }

    private static boolean isHoodBoundingBoxInTrenchZone(BoundingBox hoodBoundingBox) {
        return hoodBoundingBox.overlaps(FieldConstants.BLUE_CLOSE_TRENCH_BOUNDING_BOX)
                || hoodBoundingBox.overlaps(FieldConstants.BLUE_FAR_TRENCH_BOUNDING_BOX)
                || hoodBoundingBox.overlaps(FieldConstants.RED_CLOSE_TRENCH_BOUNDING_BOX)
                || hoodBoundingBox.overlaps(FieldConstants.RED_FAR_TRENCH_BOUNDING_BOX);
    }

    private static boolean isHoodCenterBeforeTrench(Translation2d hoodCenter) {
        return hoodCenter.getX() < FieldConstants.BLUE_CLOSE_TRENCH_MINIMUM_X
                || hoodCenter.getX() > FieldConstants.RED_CLOSE_TRENCH_MAXIMUM_X;
    }

    private static boolean isHoodCenterAfterTrench(Translation2d hoodCenter) {
        return (hoodCenter.getX() > FieldConstants.BLUE_CLOSE_TRENCH_MAXIMUM_X
                && hoodCenter.getX() < FieldConstants.RED_CLOSE_TRENCH_MINIMUM_X);
    }

    /**
     * Builds a bounding box representing the hood's footprint on the field for a given robot pose.
     * The hood is offset from the robot origin and rotated together with the robot, so a turned
     * robot produces a correctly rotated hood box (handled by the bounding box's SAT overlap check).
     *
     * @param robotPose the robot's field-relative pose
     * @return the hood's field-relative bounding box
     */
    private static BoundingBox getHoodBoundingBox(Pose2d robotPose) {
        final Transform2d hoodOffset = new Transform2d(HOOD_OFFSET_X_METERS, HOOD_OFFSET_Y_METERS, Rotation2d.kZero);
        final Pose2d hoodPose = robotPose.transformBy(hoodOffset);
        return new BoundingBox(hoodPose, HOOD_LENGTH_METERS, HOOD_WIDTH_METERS).expandedBy(HOOD_BOUNDING_BOX_EXPANSION_METERS);
    }

    private static Pose2d getRobotPose() {
        return RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
    }

    private static Pose2d getPredictedRobotPose() {
        return RobotContainer.ROBOT_POSE_ESTIMATOR.getPredictedRobotPose(TRENCH_POSE_PREDICTION_TIME_SECONDS);
    }
}