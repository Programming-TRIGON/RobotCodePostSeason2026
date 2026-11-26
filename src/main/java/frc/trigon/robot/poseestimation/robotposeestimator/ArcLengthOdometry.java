package frc.trigon.robot.poseestimation.robotposeestimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

/**
 * A custom odometry implementation that calculates robot translation by averaging the individual
 * arc lengths traveled by each swerve module.
 * This class calculates the arc displacement of a module because it provides more accurate odometry.
 * This is because the change in angle for each module is not instantaneous, causing it to move in an arc and not a
 * straight line between cycles. We do this by calculating the radius of the arc, and then determining the two radii
 * vectors making up the arc. Finally, we subtract the vectors to find the displacement vector.
 */
public class ArcLengthOdometry extends Odometry<SwerveModulePosition[]> {
    private static final double SMALL_ANGLE_THRESHOLD_RADIANS = 1e-9;

    private final SwerveDriveKinematics kinematics;
    private final Translation2d meanModuleLocation;
    private final int numberOfModules;
    private final SwerveModulePosition[] previousModulePositions;

    private Pose2d pose;
    private Rotation2d gyroOffset;
    private Rotation2d previousHeading;

    public ArcLengthOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        this(kinematics, gyroAngle, modulePositions, Pose2d.kZero);
    }

    public ArcLengthOdometry(SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d initialPose) {
        super(kinematics, gyroAngle, modulePositions, initialPose);
        this.kinematics = kinematics;
        this.numberOfModules = modulePositions.length;
        this.meanModuleLocation = calculateMeanModuleLocation(kinematics.getModules());
        this.previousModulePositions = kinematics.copy(modulePositions);

        this.pose = initialPose;
        this.gyroOffset = gyroAngle.unaryMinus().rotateBy(initialPose.getRotation());
        this.previousHeading = initialPose.getRotation();
    }

    @Override
    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        checkModuleCount(modulePositions);
        final Rotation2d heading = gyroAngle.rotateBy(gyroOffset);

        Translation2d summedModuleDisplacements = Translation2d.kZero;
        for (int i = 0; i < numberOfModules; i++)
            summedModuleDisplacements = summedModuleDisplacements.plus(
                    calculateModuleArcDisplacementFieldRelative(previousModulePositions[i], modulePositions[i], previousHeading, heading)
            );

        final Translation2d averageModuleDisplacement = summedModuleDisplacements.div(numberOfModules);
        final Translation2d robotCenterDisplacement = averageModuleDisplacement.minus(
                meanModuleLocation.rotateBy(heading).minus(meanModuleLocation.rotateBy(previousHeading))
        );

        pose = new Pose2d(pose.getTranslation().plus(robotCenterDisplacement), heading);
        kinematics.copyInto(modulePositions, previousModulePositions);
        previousHeading = heading;
        return pose;
    }

    @Override
    public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d newPose) {
        checkModuleCount(modulePositions);
        super.resetPosition(gyroAngle, modulePositions, newPose);
        pose = newPose;
        previousHeading = newPose.getRotation();
        gyroOffset = gyroAngle.unaryMinus().rotateBy(newPose.getRotation());
        kinematics.copyInto(modulePositions, previousModulePositions);
    }

    @Override
    public void resetPose(Pose2d newPose) {
        super.resetPose(newPose);
        gyroOffset = gyroOffset.rotateBy(pose.getRotation().unaryMinus()).rotateBy(newPose.getRotation());
        pose = newPose;
        previousHeading = newPose.getRotation();
    }

    @Override
    public void resetTranslation(Translation2d translation) {
        super.resetTranslation(translation);
        pose = new Pose2d(translation, pose.getRotation());
    }

    @Override
    public void resetRotation(Rotation2d rotation) {
        super.resetRotation(rotation);
        gyroOffset = gyroOffset.rotateBy(pose.getRotation().unaryMinus()).rotateBy(rotation);
        pose = new Pose2d(pose.getTranslation(), rotation);
        previousHeading = rotation;
    }

    @Override
    public Pose2d getPoseMeters() {
        return pose;
    }

    /**
     * Calculates the field-relative displacement of a single module, assuming it travelled along a circular arc.
     *
     * @param previousPosition the module's previous position
     * @param currentPosition  the module's current position
     * @param previousHeading  the robot's previous heading
     * @param currentHeading   the robot's current heading
     * @return the module's displacement relative to the field
     */
    static Translation2d calculateModuleArcDisplacementFieldRelative(SwerveModulePosition previousPosition, SwerveModulePosition currentPosition, Rotation2d previousHeading, Rotation2d currentHeading) {
        final double distanceTravelledMeters = currentPosition.distanceMeters - previousPosition.distanceMeters;
        final Rotation2d previousModuleFieldAngle = previousHeading.plus(previousPosition.angle);
        final Rotation2d currentModuleFieldAngle = currentHeading.plus(currentPosition.angle);
        final double moduleFieldAngleChangeRadians = currentModuleFieldAngle.minus(previousModuleFieldAngle).getRadians();

        final double halfAngleChangeRadians = moduleFieldAngleChangeRadians / 2;
        final double chordLengthMeters = distanceTravelledMeters * sinc(halfAngleChangeRadians);
        final Rotation2d chordDirection = previousModuleFieldAngle.plus(Rotation2d.fromRadians(halfAngleChangeRadians));
        return new Translation2d(chordLengthMeters, chordDirection);
    }

    /**
     * @return sin(x) / x, which approaches 1 as x approaches 0. When the module didn't turn this is a straight line
     */
    private static double sinc(double x) {
        if (Math.abs(x) < SMALL_ANGLE_THRESHOLD_RADIANS)
            return 1 - x * x / 6;
        return Math.sin(x) / x;
    }

    private static Translation2d calculateMeanModuleLocation(Translation2d[] moduleLocations) {
        Translation2d sum = Translation2d.kZero;
        for (Translation2d moduleLocation : moduleLocations)
            sum = sum.plus(moduleLocation);
        return sum.div(moduleLocations.length);
    }

    private void checkModuleCount(SwerveModulePosition[] modulePositions) {
        if (modulePositions.length != numberOfModules)
            throw new IllegalArgumentException("Number of modules is not consistent with number of wheel locations provided in constructor");
    }
}