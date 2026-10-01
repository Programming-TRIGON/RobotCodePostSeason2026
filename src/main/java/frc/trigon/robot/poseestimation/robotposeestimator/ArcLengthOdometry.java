package frc.trigon.robot.poseestimation.robotposeestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * A custom odometry implementation that calculates robot translation by averaging the individual
 * arc lengths traveled by each swerve module.
 * This class calculates the arc displacement of a module is because it provides more accurate odometry.
 * This is because the change in angle for each module is not instantaneous, causing it to move in an arc and not a
 * straight line between cycles. We do this by calculating the radius of the arc, and then determining the two radii
 * vectors making up the arc. Finally, we subtract the vectors to find the displacement vector.
 */
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
        for (int i = 0; i < amountOfModules; i++) {
            this.previousWheelPositions[i] = new SwerveModulePosition(modulePositions[i].distanceMeters, modulePositions[i].angle);
        }
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

        final Translation2d averagedTravelVector = calculateAveragedTravelVector(modulePositions, currentRobotAngle);

        final Twist2d twist = new Twist2d(
                averagedTravelVector.getX(),
                averagedTravelVector.getY(),
                deltaRobotHeading.getRadians()
        );

        pose = pose.exp(twist);
        previousAngle = currentRobotAngle;

        return pose;
    }

    private Translation2d calculateAveragedTravelVector(SwerveModulePosition[] modulePositions, Rotation2d currentRobotAngle) {
        Translation2d totalTravelVector = new Translation2d();

        for (int i = 0; i < amountOfModules; i++) {
            final SwerveModulePosition currentModulePosition = modulePositions[i];
            final SwerveModulePosition previousModulePosition = previousWheelPositions[i];

            final Translation2d moduleArcVector = calculateModuleTravelVector(currentModulePosition, previousModulePosition, currentRobotAngle);
            totalTravelVector = totalTravelVector.plus(moduleArcVector);

            previousModulePosition.distanceMeters = currentModulePosition.distanceMeters;
            previousModulePosition.angle = currentModulePosition.angle;
        }

        return totalTravelVector.div(amountOfModules);
    }

    private Translation2d calculateModuleTravelVector(SwerveModulePosition currentModulePosition, SwerveModulePosition previousModulePosition, Rotation2d currentRobotAngle) {
        final double deltaDistanceMeters = currentModulePosition.distanceMeters - previousModulePosition.distanceMeters;

        final Rotation2d currentModuleFieldHeading = currentRobotAngle.plus(currentModulePosition.angle);
        final Rotation2d previousModuleFieldHeading = previousAngle.plus(previousModulePosition.angle);

        final double deltaHeading = currentModuleFieldHeading.minus(previousModuleFieldHeading).getRadians();

        if (Math.abs(deltaHeading) < 1e-6)
            return new Translation2d(deltaDistanceMeters, currentModulePosition.angle);

        final double radiusMeters = deltaDistanceMeters / deltaHeading;
        final Translation2d centerToPrevious = new Translation2d(radiusMeters, previousModulePosition.angle.minus(Rotation2d.fromDegrees(90)));
        final Translation2d centerToCurrent = centerToPrevious.rotateBy(Rotation2d.fromRadians(deltaHeading));

        return centerToCurrent.minus(centerToPrevious);
    }
}