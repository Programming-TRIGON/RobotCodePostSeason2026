package frc.trigon.robot.constants;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.trigon.lib.utilities.BoundingBox;
import frc.trigon.lib.utilities.FilesHandler;
import frc.trigon.lib.utilities.flippable.Flippable;
import frc.trigon.lib.utilities.flippable.FlippablePose2d;
import frc.trigon.robot.RobotContainer;
import frc.trigon.lib.utilities.flippable.FlippableTranslation2d;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;

public class FieldConstants {
    public static final double
            FIELD_WIDTH_METERS = FlippingUtil.fieldSizeY,
            FIELD_LENGTH_METERS = FlippingUtil.fieldSizeX;
    public static final BoundingBox FIELD_BOUNDING_BOX = new BoundingBox(
            new Translation2d(0, 0),
            new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS)
    );
    private static final List<Integer> I_HATE_YOU = List.of(
            //Tags to ignore
    );

    private static final double MILLIMETERS_PER_METER = 1000.0;
    private static final boolean SHOULD_USE_HOME_TAG_LAYOUT = false;
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = createAprilTagFieldLayout();
    private static final Transform3d TAG_OFFSET = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    public static final HashMap<Integer, Pose3d> TAG_ID_TO_POSE = fieldLayoutToTagIDToPoseMap();

    public static final double ALLIANCE_ZONE_LENGTH_METERS = 4.5;
    private static final double
            BLUE_RELATIVE_DELIVERY_POSITION_X = 3.0,
            DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS = 2.2;
    public static final FlippableTranslation2d
            HUB_POSITION = new FlippableTranslation2d(TAG_ID_TO_POSE.get(26).getX() + (Units.inchesToMeters(47) / 2), FIELD_WIDTH_METERS / 2, true),
            RIGHT_DELIVERY_POSITION = new FlippableTranslation2d(BLUE_RELATIVE_DELIVERY_POSITION_X, (FIELD_WIDTH_METERS / 2) - DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS, true),
            LEFT_DELIVERY_POSITION = new FlippableTranslation2d(BLUE_RELATIVE_DELIVERY_POSITION_X, (FIELD_WIDTH_METERS / 2) + DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS, true);

    private static AprilTagFieldLayout createAprilTagFieldLayout() {
        try {
            return SHOULD_USE_HOME_TAG_LAYOUT ?
                    new AprilTagFieldLayout(FilesHandler.DEPLOY_PATH + "field_calibration.json") :
                    AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private static HashMap<Integer, Pose3d> fieldLayoutToTagIDToPoseMap() {
        final HashMap<Integer, Pose3d> tagIDToPose = new HashMap<>();
        for (AprilTag aprilTag : APRIL_TAG_FIELD_LAYOUT.getTags())
            if (!I_HATE_YOU.contains(aprilTag.ID))
                tagIDToPose.put(aprilTag.ID, aprilTag.pose.transformBy(TAG_OFFSET));

        return tagIDToPose;
    }

    /**
     * Mirrors a FlippablePose2d across the field's Y-axis centerline.
     */
    public static FlippablePose2d mirror(FlippablePose2d pose) {
        final Pose2d basePose = pose.getBlueObject();
        return new FlippablePose2d(
                basePose.getX(),
                FIELD_WIDTH_METERS - basePose.getY(),
                Rotation2d.fromDegrees(-basePose.getRotation().getDegrees()),
                true
        );
    }

    public static boolean isRobotInAllianceZone() {
        return isPoseInAllianceZone(RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation());
    }

    public static boolean isPoseInAllianceZone(Translation2d pose) {
        if (pose == null)
            return false;
        if (Flippable.isRedAlliance())
            return pose.getX() > FieldConstants.FIELD_LENGTH_METERS - FieldConstants.ALLIANCE_ZONE_LENGTH_METERS;
        return pose.getX() < FieldConstants.ALLIANCE_ZONE_LENGTH_METERS;
    }

    public static boolean isRight() {
        if (Flippable.isRedAlliance())
            return RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation().getY() > FieldConstants.FIELD_WIDTH_METERS / 2;
        return RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose().getTranslation().getY() < FieldConstants.FIELD_WIDTH_METERS / 2;
    }

    private static final double
            HOOD_OFFSET_X_METERS = -150 / MILLIMETERS_PER_METER,
            HOOD_OFFSET_Y_METERS = 484.6 / MILLIMETERS_PER_METER,
            HOOD_WIDTH_METERS = 619 / MILLIMETERS_PER_METER,
            HOOD_LENGTH_METERS = 256 / MILLIMETERS_PER_METER,
            HOOD_BOUNDING_BOX_EXPANSION_METERS = 0.1;

    private static final double TRENCH_POSE_PREDICTION_TIME_SECONDS = 0.2;

    private static final double
            BLUE_RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_MIN_X = 4.000,
            BLUE_RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_MAX_X = 5.223,
            BLUE_RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_MIN_Y = 0.000,
            BLUE_RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_MAX_Y = 1.28;

    private static final double
            BLUE_LEFT_HOOD_ELEVATION_RESTRICTED_ZONE_MIN_X = BLUE_RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_MIN_X,
            BLUE_LEFT_HOOD_ELEVATION_RESTRICTED_ZONE_MAX_X = BLUE_RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_MAX_X,
            BLUE_LEFT_HOOD_ELEVATION_RESTRICTED_ZONE_MIN_Y = FIELD_WIDTH_METERS - BLUE_RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_MAX_Y,
            BLUE_LEFT_HOOD_ELEVATION_RESTRICTED_ZONE_MAX_Y = FIELD_WIDTH_METERS - BLUE_RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_MIN_Y;

    private static final FlippableTranslation2d
            RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_CORNER_A = new FlippableTranslation2d(
            BLUE_RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_MIN_X,
            BLUE_RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_MIN_Y,
            true
    ),
            RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_CORNER_B = new FlippableTranslation2d(
                    BLUE_RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_MAX_X,
                    BLUE_RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_MAX_Y,
                    true
            ),
            LEFT_HOOD_ELEVATION_RESTRICTED_ZONE_CORNER_A = new FlippableTranslation2d(
                    BLUE_LEFT_HOOD_ELEVATION_RESTRICTED_ZONE_MIN_X,
                    BLUE_LEFT_HOOD_ELEVATION_RESTRICTED_ZONE_MIN_Y,
                    true
            ),
            LEFT_HOOD_ELEVATION_RESTRICTED_ZONE_CORNER_B = new FlippableTranslation2d(
                    BLUE_LEFT_HOOD_ELEVATION_RESTRICTED_ZONE_MAX_X,
                    BLUE_LEFT_HOOD_ELEVATION_RESTRICTED_ZONE_MAX_Y,
                    true
            );

    private static BoundingBox getRightHoodElevationRestrictedZoneBoundingBox() {
        return new BoundingBox(
                RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_CORNER_A.get(),
                RIGHT_HOOD_ELEVATION_RESTRICTED_ZONE_CORNER_B.get()
        );
    }

    private static BoundingBox getLeftHoodElevationRestrictedZoneBoundingBox() {
        return new BoundingBox(
                LEFT_HOOD_ELEVATION_RESTRICTED_ZONE_CORNER_A.get(),
                LEFT_HOOD_ELEVATION_RESTRICTED_ZONE_CORNER_B.get()
        );
    }

    private static BoundingBox currentHoodBoundingBox = null;
    private static BoundingBox predictedHoodBoundingBox = null;

    /**
     * Recalculates the cached current and predicted hood bounding boxes and logs the trench detection state.
     * Should be called once per loop, before the trench trigger is polled.
     */
    public static void updateTrenchDetection() {
        currentHoodBoundingBox = getHoodBoundingBox(getRobotPose());
        predictedHoodBoundingBox = getHoodBoundingBox(getPredictedRobotPose());

        logTrenchBoundingBoxes();
        Logger.recordOutput("Zones/IsHoodInTrenchZone", isHoodInTrenchZone());
    }

    /**
     * Checks whether the hood overlaps a trench anywhere along its predicted path of travel.
     * Instead of sampling only the current and predicted poses, this checks the entire swept area
     * between them, so a fast pass-through where the hood is before the trench now and past it in
     * the prediction is still caught.
     *
     * @return whether the hood is in or passing through a trench
     */
    public static boolean isHoodInTrenchZone() {
        if (currentHoodBoundingBox == null || predictedHoodBoundingBox == null)
            return false;
        return isHoodBoundingBoxInRestrictedElevationZone(getSweptHoodBoundingBox());
    }

    private static BoundingBox getSweptHoodBoundingBox() {
        final Translation2d
                currentCenter = currentHoodBoundingBox.getCenter().getTranslation(),
                predictedCenter = predictedHoodBoundingBox.getCenter().getTranslation();
        final Translation2d travel = predictedCenter.minus(currentCenter);
        final Rotation2d travelDirection = travel.getNorm() < 1e-6 ? currentHoodBoundingBox.getCenter().getRotation() : travel.getAngle();
        final Translation2d sweptCenter = currentCenter.plus(travel.div(2));

        final double
                sweptLength = travel.getNorm() + HOOD_LENGTH_METERS,
                sweptWidth = HOOD_WIDTH_METERS;

        return new BoundingBox(new Pose2d(sweptCenter, travelDirection), sweptLength, sweptWidth)
                .expandedBy(HOOD_BOUNDING_BOX_EXPANSION_METERS);
    }

    private static boolean isHoodBoundingBoxInRestrictedElevationZone(BoundingBox hoodBoundingBox) {
        return hoodBoundingBox.overlaps(getRightHoodElevationRestrictedZoneBoundingBox())
                || hoodBoundingBox.overlaps(getLeftHoodElevationRestrictedZoneBoundingBox());
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

    private static Pose2d getRobotPose() {return RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();}

    private static Pose2d getPredictedRobotPose() {return RobotContainer.ROBOT_POSE_ESTIMATOR.getPredictedRobotPose(TRENCH_POSE_PREDICTION_TIME_SECONDS);}

    private static void logTrenchBoundingBoxes() {
        getRightHoodElevationRestrictedZoneBoundingBox().log("Zones/RightHoodElevationRestrictedZone");
        getLeftHoodElevationRestrictedZoneBoundingBox().log("Zones/LeftHoodElevationRestrictedZone");
        getSweptHoodBoundingBox().log("Zones/SweptHoodBoundingBox");
        currentHoodBoundingBox.log("Zones/HoodBoundingBox");
        predictedHoodBoundingBox.log("Zones/PredictedHoodBoundingBox");
    }
}