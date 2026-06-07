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
            HOOD_OFFSET_X_METERS = -150 / 1000.0,
            HOOD_OFFSET_Y_METERS = 484.6 / 1000.0,
            HOOD_WIDTH_METERS = 619 / 1000.0,
            HOOD_LENGTH_METERS = 256 / 1000.0;


    private static final double TRENCH_POSE_PREDICTION_TIME_SECONDS = 0.2;

    private static final double
            BLUE_RIGHT_TRENCH_ARM_MIN_X = 4.000,
            BLUE_RIGHT_TRENCH_ARM_MAX_X = 5.223,
            BLUE_RIGHT_TRENCH_ARM_MIN_Y = 0.000,
            BLUE_RIGHT_TRENCH_ARM_MAX_Y = 1.28;

    private static final double
            BLUE_LEFT_TRENCH_ARM_MIN_X = BLUE_RIGHT_TRENCH_ARM_MIN_X,
            BLUE_LEFT_TRENCH_ARM_MAX_X = BLUE_RIGHT_TRENCH_ARM_MAX_X,
            BLUE_LEFT_TRENCH_ARM_MIN_Y = FIELD_WIDTH_METERS - BLUE_RIGHT_TRENCH_ARM_MAX_Y,
            BLUE_LEFT_TRENCH_ARM_MAX_Y = FIELD_WIDTH_METERS - BLUE_RIGHT_TRENCH_ARM_MIN_Y;

    private static final double
            RED_LEFT_TRENCH_ARM_MIN_X = FIELD_LENGTH_METERS - BLUE_RIGHT_TRENCH_ARM_MAX_X,
            RED_LEFT_TRENCH_ARM_MAX_X = FIELD_LENGTH_METERS - BLUE_RIGHT_TRENCH_ARM_MIN_X,
            RED_LEFT_TRENCH_ARM_MIN_Y = BLUE_RIGHT_TRENCH_ARM_MIN_Y,
            RED_LEFT_TRENCH_ARM_MAX_Y = BLUE_RIGHT_TRENCH_ARM_MAX_Y;

    private static final double
            RED_RIGHT_TRENCH_ARM_MIN_X = FIELD_LENGTH_METERS - BLUE_RIGHT_TRENCH_ARM_MAX_X,
            RED_RIGHT_TRENCH_ARM_MAX_X = FIELD_LENGTH_METERS - BLUE_RIGHT_TRENCH_ARM_MIN_X,
            RED_RIGHT_TRENCH_ARM_MIN_Y = BLUE_LEFT_TRENCH_ARM_MIN_Y,
            RED_RIGHT_TRENCH_ARM_MAX_Y = BLUE_LEFT_TRENCH_ARM_MAX_Y;

    public static final BoundingBox
            BLUE_ALLIANCE_RIGHT_TRENCH_BOUNDING_BOX = new BoundingBox(
            new Translation2d(BLUE_RIGHT_TRENCH_ARM_MIN_X, BLUE_RIGHT_TRENCH_ARM_MIN_Y),
            new Translation2d(BLUE_RIGHT_TRENCH_ARM_MAX_X, BLUE_RIGHT_TRENCH_ARM_MAX_Y)
    ),
            BLUE_ALLIANCE_LEFT_TRENCH_BOUNDING_BOX = new BoundingBox(
                    new Translation2d(BLUE_LEFT_TRENCH_ARM_MIN_X, BLUE_LEFT_TRENCH_ARM_MIN_Y),
                    new Translation2d(BLUE_LEFT_TRENCH_ARM_MAX_X, BLUE_LEFT_TRENCH_ARM_MAX_Y)
            ),
            RED_ALLIANCE_RIGHT_TRENCH_BOUNDING_BOX = new BoundingBox(
                    new Translation2d(RED_RIGHT_TRENCH_ARM_MIN_X, RED_RIGHT_TRENCH_ARM_MIN_Y),
                    new Translation2d(RED_RIGHT_TRENCH_ARM_MAX_X, RED_RIGHT_TRENCH_ARM_MAX_Y)
            ),
            RED_ALLIANCE_LEFT_TRENCH_BOUNDING_BOX = new BoundingBox(
                    new Translation2d(RED_LEFT_TRENCH_ARM_MIN_X, RED_LEFT_TRENCH_ARM_MIN_Y),
                    new Translation2d(RED_LEFT_TRENCH_ARM_MAX_X, RED_LEFT_TRENCH_ARM_MAX_Y)
            );

    /**
     * Checks whether the hood is currently over a trench, or is predicted to be
     * over a trench within the prediction time based on the robot's velocity.
     * The hood should be lowered while this is true so that it does not hit the trench.
     *
     * @return whether the hood is in or approaching a trench
     */
    public static boolean isHoodInTrenchZone() {
        return isHoodBoundingBoxInTrenchZone(getHoodBoundingBox(getRobotPose()))
                || isHoodBoundingBoxInTrenchZone(getHoodBoundingBox(getPredictedRobotPose()));
    }

    private static boolean isHoodBoundingBoxInTrenchZone(BoundingBox hoodBoundingBox) {
        return hoodBoundingBox.overlaps(BLUE_ALLIANCE_RIGHT_TRENCH_BOUNDING_BOX)
                || hoodBoundingBox.overlaps(BLUE_ALLIANCE_LEFT_TRENCH_BOUNDING_BOX)
                || hoodBoundingBox.overlaps(RED_ALLIANCE_RIGHT_TRENCH_BOUNDING_BOX)
                || hoodBoundingBox.overlaps(RED_ALLIANCE_LEFT_TRENCH_BOUNDING_BOX);
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
            return new BoundingBox(hoodPose, HOOD_LENGTH_METERS, HOOD_WIDTH_METERS).expandedBy(0.1);
    }

    private static Pose2d getRobotPose() {
        return RobotContainer.ROBOT_POSE_ESTIMATOR.getEstimatedRobotPose();
    }

    private static Pose2d getPredictedRobotPose() {
        return RobotContainer.ROBOT_POSE_ESTIMATOR.getPredictedRobotPose(TRENCH_POSE_PREDICTION_TIME_SECONDS);
    }

    /**
     * Logs all trench arm bounding boxes and the current hood bounding box for visualization in AdvantageScope.
     */
    public static void logTrenchBoundingBoxes() {
        BLUE_ALLIANCE_RIGHT_TRENCH_BOUNDING_BOX.log("Zones/BlueRightTrenchArm");
        BLUE_ALLIANCE_LEFT_TRENCH_BOUNDING_BOX.log("Zones/BlueLeftTrenchArm");
        RED_ALLIANCE_RIGHT_TRENCH_BOUNDING_BOX.log("Zones/RedRightTrenchArm");
        RED_ALLIANCE_LEFT_TRENCH_BOUNDING_BOX.log("Zones/RedLeftTrenchArm");

        getHoodBoundingBox(getRobotPose()).log("Zones/HoodBoundingBox");
        getHoodBoundingBox(getPredictedRobotPose()).log("Zones/PredictedHoodBoundingBox");

        Logger.recordOutput("Zones/IsHoodInTrenchZone", isHoodInTrenchZone());
    }
}