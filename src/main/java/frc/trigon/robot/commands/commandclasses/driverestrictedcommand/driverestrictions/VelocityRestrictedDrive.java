package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;

/**
 * A restriction that limits the robot's maximum linear and rotational velocity.
 * Used to slow down the robot for precise robot movement.
 */
public class VelocityRestrictedDrive implements DriveRestriction {
    private final double maximumTranslationVelocityPower;
    private final double maximumRotationVelocityPower;

    /**
     * A restriction that limits the maximum linear and rotational velocity.
     *
     * @param maximumTranslationVelocityMetersPerSecond maximum linear velocity
     * @param maximumRotationVelocityRadiansPerSecond   maximum rotational velocity
     */
    public VelocityRestrictedDrive(double maximumTranslationVelocityMetersPerSecond, Rotation2d maximumRotationVelocityRadiansPerSecond) {
        this.maximumTranslationVelocityPower = MathUtil.clamp(maximumTranslationVelocityMetersPerSecond / SwerveConstants.MAXIMUM_SPEED_METERS_PER_SECOND, 0, 1);
        this.maximumRotationVelocityPower = MathUtil.clamp(maximumRotationVelocityRadiansPerSecond.getRadians() / SwerveConstants.MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND, 0, 1);
    }

    @Override
    public Translation2d applyTranslationRestriction(Translation2d targetTranslation) {
        final Translation2d scaledRobotTargetTranslation = targetTranslation.times(maximumTranslationVelocityPower);
        final double scaledRobotTargetTranslationMagnitude = scaledRobotTargetTranslation.getNorm();

        if (scaledRobotTargetTranslationMagnitude > maximumTranslationVelocityPower)
            return scaledRobotTargetTranslation.times(maximumTranslationVelocityPower / scaledRobotTargetTranslationMagnitude);
        return scaledRobotTargetTranslation;
    }

    @Override
    public double applyRotationRestriction(double targetRotation) {
        return targetRotation * maximumRotationVelocityPower;
    }
}