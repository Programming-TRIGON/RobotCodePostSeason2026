package frc.trigon.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCamera;
import frc.trigon.robot.poseestimation.apriltagcamera.AprilTagCameraConstants;
import frc.trigon.robot.poseestimation.robotposeestimator.StandardDeviations;

public class CameraConstants {
    private static final StandardDeviations APRIL_TAG_CAMERA_STANDARD_DEVIATIONS = new StandardDeviations(
            0.016,
            0.01
    );
    private static final Transform3d
            ROBOT_TO_BACK_LEFT_APRIL_TAG_CAMERA = new Transform3d(
                    new Translation3d(
                        -0.356,
                        0.246,
                        0.416
                    ),
                    new Rotation3d(
                        Units.degreesToRadians(8.956),
                        Units.degreesToRadians(180 - 30),
                        Units.degreesToRadians(180 - 20)
                    )
            ),

            ROBOT_TO_BACK_RIGHT_RIGHT_APRIL_TAG_CAMERA = new Transform3d(
                    new Translation3d(
                            -0.356,
                            -0.246,
                            0.416
                    ),
                    new Rotation3d(
                            Units.degreesToRadians(8.956),
                            Units.degreesToRadians(180 - 30),
                            Units.degreesToRadians(180 + 20)
                    )
            ),
            ROBOT_TO_BACK_SIDE_LEFT_APRIL_TAG_CAMERA = new Transform3d(
                    new Translation3d(
                            -0.319,
                            0.291,
                            0.227
                    ),
                    new Rotation3d(
                            0,
                            Units.degreesToRadians(30),
                            Units.degreesToRadians(-70)
                    )
            ),
            ROBOT_TO_BACK_SIDE_RIGHT_APRIL_TAG_CAMERA = new Transform3d(
                    new Translation3d(
                            -0.319,
                            -0.291,
                            0.227
                    ),
                    new Rotation3d(
                            0,
                            Units.degreesToRadians(30),
                            Units.degreesToRadians(70)
                    )
            );

    public static final AprilTagCamera
            BACK_LEFT_APRIL_TAG_CAMERA = new AprilTagCamera(
            AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
            "BackLeftAprilTagCamera",
            ROBOT_TO_BACK_LEFT_APRIL_TAG_CAMERA,
            APRIL_TAG_CAMERA_STANDARD_DEVIATIONS
            ),
            BACK_RIGHT_APRIL_TAG_CAMERA = new AprilTagCamera(
                    AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
                    "BackRightAprilTagCamera",
                    ROBOT_TO_BACK_RIGHT_RIGHT_APRIL_TAG_CAMERA,
                    APRIL_TAG_CAMERA_STANDARD_DEVIATIONS
            ),
            BACK_SIDE_LEFT_APRIL_TAG_CAMERA = new AprilTagCamera(
                    AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
                    "BackSideLeftAprilTagCamera",
                    ROBOT_TO_BACK_SIDE_LEFT_APRIL_TAG_CAMERA,
                    APRIL_TAG_CAMERA_STANDARD_DEVIATIONS
            ),
            BACK_SIDE_RIGHT_APRIL_TAG_CAMERA = new AprilTagCamera(
                    AprilTagCameraConstants.AprilTagCameraType.PHOTON_CAMERA,
                    "BackSideRightAprilTagCamera",
                    ROBOT_TO_BACK_SIDE_RIGHT_APRIL_TAG_CAMERA,
                    APRIL_TAG_CAMERA_STANDARD_DEVIATIONS
            );
}