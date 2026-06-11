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

    public static final double MILLIMETERS_TO_METERS = 1000.0;
    public static final double ALLIANCE_ZONE_LENGTH_METERS = 4.5;
    private static final double
            BLUE_RELATIVE_DELIVERY_POSITION_X = 3.0,
            DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS = 2.2;
    public static final double
            HALF_SIZE_OF_HUB_CENTER_X = 1.19 / 2.0,
            EXTRA_HUB_WIDTH = 0.25;
    public static final FlippableTranslation2d
            HUB_POSITION = new FlippableTranslation2d(TAG_ID_TO_POSE.get(26).getX() + (Units.inchesToMeters(47) / 2), FIELD_WIDTH_METERS / 2, true),
            RIGHT_DELIVERY_POSITION = new FlippableTranslation2d(BLUE_RELATIVE_DELIVERY_POSITION_X, (FIELD_WIDTH_METERS / 2) - DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS, true),
            LEFT_DELIVERY_POSITION = new FlippableTranslation2d(BLUE_RELATIVE_DELIVERY_POSITION_X, (FIELD_WIDTH_METERS / 2) + DELIVERY_POSITION_Y_OFFSET_FROM_CENTER_METERS, true);

    /**
     * The trench bounding box coordinates in meters.
     * LEFT trench runs along the south wall (Y ~0) — left side from the driver's perspective.
     * RIGHT trench mirrors it along the other wall — right side from the driver's perspective.
     */
    public static final double
            RIGHT_TRENCH_MINIMUM_X = 4.000,
            RIGHT_TRENCH_MAXIMUM_X = 5.223,
            RIGHT_TRENCH_MINIMUM_Y = 0.000,
            RIGHT_TRENCH_MAXIMUM_Y = 1.28;
    private static final double
            LEFT_TRENCH_MINIMUM_X = RIGHT_TRENCH_MINIMUM_X,
            LEFT_TRENCH_MAXIMUM_X = RIGHT_TRENCH_MAXIMUM_X,
            LEFT_TRENCH_MINIMUM_Y = FIELD_WIDTH_METERS - RIGHT_TRENCH_MAXIMUM_Y,
            LEFT_TRENCH_MAXIMUM_Y = FIELD_WIDTH_METERS - RIGHT_TRENCH_MINIMUM_Y;
    private static final FlippableTranslation2d
            FAR_RIGHT_TRENCH_CORNER_A = new FlippableTranslation2d(RIGHT_TRENCH_MINIMUM_X, RIGHT_TRENCH_MINIMUM_Y, true),
            FAR_LEFT_TRENCH_CORNER_A = new FlippableTranslation2d(LEFT_TRENCH_MINIMUM_X, LEFT_TRENCH_MINIMUM_Y,true),
            FAR_RIGHT_TRENCH_CORNER_B = new FlippableTranslation2d(RIGHT_TRENCH_MAXIMUM_X, RIGHT_TRENCH_MAXIMUM_Y,true),
            FAR_LEFT_TRENCH_CORNER_B = new FlippableTranslation2d(LEFT_TRENCH_MAXIMUM_X, LEFT_TRENCH_MAXIMUM_Y, true),
            CLOSE_RIGHT_TRENCH_CORNER_A = new FlippableTranslation2d(FIELD_LENGTH_METERS - RIGHT_TRENCH_MAXIMUM_X, RIGHT_TRENCH_MINIMUM_Y, true),
            CLOSE_LEFT_TRENCH_CORNER_A = new FlippableTranslation2d(FIELD_LENGTH_METERS - LEFT_TRENCH_MAXIMUM_X, LEFT_TRENCH_MINIMUM_Y, true),
            COSE_RIGHT_TRENCH_CORNER_B = new FlippableTranslation2d(FIELD_LENGTH_METERS - RIGHT_TRENCH_MINIMUM_X, RIGHT_TRENCH_MAXIMUM_Y, true),
            CLOSE_LEFT_TRENCH_CORNER_B = new FlippableTranslation2d(FIELD_LENGTH_METERS - LEFT_TRENCH_MINIMUM_X, LEFT_TRENCH_MAXIMUM_Y, true);

    public static final BoundingBox
            FAR_RIGHT_TRENCH_BOUNDING_BOX = new BoundingBox(
            FAR_RIGHT_TRENCH_CORNER_A.get(),
            FAR_RIGHT_TRENCH_CORNER_B.get()
    ),
            FAR_LEFT_TRENCH_BOUNDING_BOX = new BoundingBox(
                    FAR_LEFT_TRENCH_CORNER_A.get(),
                    FAR_LEFT_TRENCH_CORNER_B.get()
            ),
            CLOSE_BLUE_RIGHT_TRENCH_BOUNDING_BOX = new BoundingBox(
                    CLOSE_RIGHT_TRENCH_CORNER_A.get(),
                    COSE_RIGHT_TRENCH_CORNER_B.get()
            ),
            CLOSE_BLUE_LEFT_TRENCH_BOUNDING_BOX = new BoundingBox(
                    CLOSE_LEFT_TRENCH_CORNER_A.get(),
                    CLOSE_LEFT_TRENCH_CORNER_B.get()
            );

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
}