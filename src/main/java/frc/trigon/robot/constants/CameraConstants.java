package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.robotposeestimator.StandardDeviations;

public class CameraConstants {
    private static final StandardDeviations APRIL_TAG_CAMERA_STANDARD_DEVIATIONS = new StandardDeviations(
            0.016,
            0.01
    );
    private static final Transform3d
            ROBOT_TO_RIGHT_APRIL_TAG_CAMERA = new Transform3d(
            new Translation3d(),
            new Rotation3d()
    ), //TODO: get actual transforms
            ROBOT_TO_LEFT_APRIL_TAG_CAMERA = new Transform3d(
                    new Translation3d(),
                    new Rotation3d()
            ); //TODO: get actual transforms
    public static final AprilTagCamera
            RIGHT_APRIL_TAG_CAMERA = new AprilTagCamera(
            AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
            "RightAprilTagCamera",
            ROBOT_TO_RIGHT_APRIL_TAG_CAMERA,
            APRIL_TAG_CAMERA_STANDARD_DEVIATIONS
    ),
            LEFT_APRIL_TAG_CAMERA = new AprilTagCamera(
                    AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
                    "LeftAprilTagCamera",
                    ROBOT_TO_LEFT_APRIL_TAG_CAMERA,
                    APRIL_TAG_CAMERA_STANDARD_DEVIATIONS
            );
}