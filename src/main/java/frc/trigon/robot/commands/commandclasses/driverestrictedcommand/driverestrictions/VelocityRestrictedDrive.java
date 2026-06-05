package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;

/**
 * A restriction that limits the linear and rotational velocity of the robot.
 */
public class VelocityRestrictedDrive implements DriveRestriction {
    private final double maximumTranslationVelocityMetersPerSecond;
    private final double maximumRotationVelocityRadiansPerSecond;

    /**
     * A restriction that limits the maximum linear and rotational velocity.
     *
     * @param maximumTranslationVelocityMetersPerSecond Maximum linear velocity, has to be between 0 and 1.
     * @param maximumRotationVelocityRadiansPerSecond       Maximum rotational velocity, has to between 0 and 1.
     */
    public VelocityRestrictedDrive(double maximumTranslationVelocityMetersPerSecond, double maximumRotationVelocityRadiansPerSecond) {
        this.maximumTranslationVelocityMetersPerSecond = maximumTranslationVelocityMetersPerSecond / SwerveConstants.MAXIMUM_SPEED_METERS_PER_SECOND;
        this.maximumRotationVelocityRadiansPerSecond = maximumRotationVelocityRadiansPerSecond / SwerveConstants.MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND;
    }

    @Override
    public Translation2d applyRestrictionToTranslation(Translation2d targetTranslation) {
        final double translationMagnitude = targetTranslation.getNorm();
        if (translationMagnitude > maximumTranslationVelocityMetersPerSecond) {
            return targetTranslation.times(maximumTranslationVelocityMetersPerSecond / translationMagnitude);
        }
        return targetTranslation;
    }

    @Override
    public double applyRestrictionToRotation(double targetRotation) {
        return MathUtil.clamp(targetRotation, -maximumRotationVelocityRadiansPerSecond, maximumRotationVelocityRadiansPerSecond);
    }
}