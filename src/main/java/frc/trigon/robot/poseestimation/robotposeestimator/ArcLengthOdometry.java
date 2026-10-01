package frc.trigon.robot.poseestimation.robotposeestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class ArcLengthOdometry {
    private final int amountOfModules;
    private final Rotation2d gyroOffset;
    private Pose2d pose;
    private Rotation2d previousAngle;
    private final SwerveModulePosition[] previousWheelPositions;

    /**
     * Constructs a custom Arc Length Odometry class.
     */
    public ArcLengthOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPose) {
        this.amountOfModules = modulePositions.length;
        this.pose = initialPose;
        this.gyroOffset = pose.getRotation().minus(gyroAngle);
        this.previousAngle = initialPose.getRotation();

        this.previousWheelPositions = new SwerveModulePosition[amountOfModules];
        for (int i = 0; i < amountOfModules; i++)
            this.previousWheelPositions[i] = new SwerveModulePosition(modulePositions[i].distanceMeters, modulePositions[i].angle);
    }

    public void resetPose(Pose2d poseMeters) {
        this.pose = poseMeters;
        this.previousAngle = poseMeters.getRotation();
    }

    public Pose2d getPose() {
        return pose;
    }

    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        final Rotation2d currentRobotAngle = gyroAngle.plus(gyroOffset);
        final Rotation2d deltaRobotHeading = currentRobotAngle.minus(previousAngle);

        Translation2d averagedTravelVector = new Translation2d();

        for (int i = 0; i < amountOfModules; i++) {
            final SwerveModulePosition currentModulePosition = modulePositions[i];
            final SwerveModulePosition previousModulePosition = previousWheelPositions[i];

            final double deltaDistanceMeters = currentModulePosition.distanceMeters - previousModulePosition.distanceMeters;

            final Rotation2d currentModuleFieldHeading = currentRobotAngle.plus(currentModulePosition.angle);
            final Rotation2d previousModuleFieldHeading = previousAngle.plus(previousModulePosition.angle);

            final double deltaHeading = currentModuleFieldHeading.minus(previousModuleFieldHeading).getRadians();

            Translation2d arcTravelledVector;

            if (Math.abs(deltaHeading) < 1e-6)
                arcTravelledVector = new Translation2d(deltaDistanceMeters, currentModulePosition.angle);
            else {
                final double radiusMeters = deltaDistanceMeters / deltaHeading;
                final Translation2d centerToPrevious = new Translation2d(radiusMeters, previousModulePosition.angle.minus(Rotation2d.fromDegrees(90)));
                final Translation2d centerToCurrent = centerToPrevious.rotateBy(Rotation2d.fromRadians(deltaHeading));

                arcTravelledVector = centerToCurrent.minus(centerToPrevious);
            }

            averagedTravelVector = averagedTravelVector.plus(arcTravelledVector);

            previousModulePosition.distanceMeters = currentModulePosition.distanceMeters;
            previousModulePosition.angle = currentModulePosition.angle;
        }

        averagedTravelVector = averagedTravelVector.div(amountOfModules);

        Twist2d twist = new Twist2d(
                averagedTravelVector.getX(),
                averagedTravelVector.getY(),
                deltaRobotHeading.getRadians()
        );

        pose = pose.exp(twist);
        previousAngle = currentRobotAngle;

        return pose;
    }
}