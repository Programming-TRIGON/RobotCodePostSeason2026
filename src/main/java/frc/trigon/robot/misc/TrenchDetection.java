package frc.trigon.robot.misc;

import edu.wpi.first.math.geometry.*;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.robot.RobotContainer;
import frc.trigon.robot.constants.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class TrenchDetection {
    private static final double
            HOOD_OFFSET_X_METERS = -150 / FieldConstants.MILLIMETERS_TO_METERS,
            HOOD_OFFSET_Y_METERS = 484.6 / FieldConstants.MILLIMETERS_TO_METERS,
            HOOD_WIDTH_METERS = 619 / FieldConstants.MILLIMETERS_TO_METERS,
            HOOD_LENGTH_METERS = 256 / FieldConstants.MILLIMETERS_TO_METERS,
            HOOD_BOUNDING_BOX_EXPANSION_METERS = 0.1;
    private static final double TRENCH_POSE_PREDICTION_TIME_SECONDS = 0.2;
    private static final Transform2d HOOD_OFFSET = new Transform2d(HOOD_OFFSET_X_METERS, HOOD_OFFSET_Y_METERS, Rotation2d.kZero);

    /**
     * Checks whether the hood is currently in, will be in, or is passing through a trench zone.
     * The pass-through checks catch the case where the robot moves fast enough to cross the entire
     * trench between two samples, so neither the current nor predicted position overlaps the trench.
     *
     * @return whether the hood is in or passing through a trench
     */
    public static boolean isHoodInTrenchZone() {
        return isHoodInTrenchZone(getHoodBoundingBox(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose()), getHoodBoundingBox(RobotContainer.ROBOT_POSE_ESTIMATOR.getPredictedRobotPose(TRENCH_POSE_PREDICTION_TIME_SECONDS)));
    }

    private static boolean isHoodInTrenchZone(BoundingBox currentHoodBox, BoundingBox predictedHoodBox) {
        return isBoundingBoxInTrenchZone(currentHoodBox)
                || isBoundingBoxInTrenchZone(predictedHoodBox)
                || (isHoodCenterBeforeTrench(currentHoodBox.getCenter().getTranslation()) && isHoodCenterAfterTrench(predictedHoodBox.getCenter().getTranslation()))
                || (isHoodCenterAfterTrench(currentHoodBox.getCenter().getTranslation()) && isHoodCenterBeforeTrench(predictedHoodBox.getCenter().getTranslation()));
    }

    /**
     * Logs all trench bounding boxes and the current hood bounding box for visualization in AdvantageScope.
     */
    public static void logTrenchBoundingBoxes() {
        final BoundingBox
                currentHoodBox = getHoodBoundingBox(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose()),
                predictedHoodBox = getHoodBoundingBox(RobotContainer.ROBOT_POSE_ESTIMATOR.getPredictedRobotPose(TRENCH_POSE_PREDICTION_TIME_SECONDS));

        FieldConstants.BLUE_RIGHT_TRENCH_BOUNDING_BOX.log("Zones/BlueRightTrench");
        FieldConstants.BLUE_LEFT_TRENCH_BOUNDING_BOX.log("Zones/BlueLeftTrench");
        FieldConstants.RED_RIGHT_TRENCH_BOUNDING_BOX.log("Zones/RedRightTrench");
        FieldConstants.RED_LEFT_TRENCH_BOUNDING_BOX.log("Zones/RedLeftTrench");
        currentHoodBox.log("Zones/HoodBoundingBox");
        predictedHoodBox.log("Zones/PredictedHoodBoundingBox");
        Logger.recordOutput("Zones/IsHoodInTrenchZone", isHoodInTrenchZone(currentHoodBox, predictedHoodBox));
    }

    private static boolean isBoundingBoxInTrenchZone(BoundingBox hoodBoundingBox) {
        return hoodBoundingBox.overlaps(FieldConstants.BLUE_RIGHT_TRENCH_BOUNDING_BOX)
                || hoodBoundingBox.overlaps(FieldConstants.BLUE_LEFT_TRENCH_BOUNDING_BOX)
                || hoodBoundingBox.overlaps(FieldConstants.RED_RIGHT_TRENCH_BOUNDING_BOX)
                || hoodBoundingBox.overlaps(FieldConstants.RED_LEFT_TRENCH_BOUNDING_BOX);
    }

    private static boolean isHoodCenterBeforeTrench(Translation2d hoodCenter) {
        final double redTrenchMaximumX = FieldConstants.FIELD_LENGTH_METERS - FieldConstants.LEFT_TRENCH_MINIMUM_X;
        return hoodCenter.getX() < FieldConstants.LEFT_TRENCH_MINIMUM_X
                || hoodCenter.getX() > redTrenchMaximumX;
    }

    private static boolean isHoodCenterAfterTrench(Translation2d hoodCenter) {
        final double redTrenchMinimumX = FieldConstants.FIELD_LENGTH_METERS - FieldConstants.LEFT_TRENCH_MAXIMUM_X;
        return hoodCenter.getX() > FieldConstants.LEFT_TRENCH_MAXIMUM_X
                && hoodCenter.getX() < redTrenchMinimumX;
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
        final Pose2d hoodPose = robotPose.transformBy(HOOD_OFFSET);
        return new BoundingBox(hoodPose, HOOD_LENGTH_METERS, HOOD_WIDTH_METERS).expandedBy(HOOD_BOUNDING_BOX_EXPANSION_METERS);
    }
}