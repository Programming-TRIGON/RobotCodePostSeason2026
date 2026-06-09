package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;

/**
 * A restriction that caps the robot's maximum linear and rotational velocity.
 * Used for precise actions like scoring approaches or fine alignment, where full driver speed would overshoot.
 */
public class VelocityRestrictedDrive implements DriveRestriction {
    private final double maximumTranslationVelocity;
    private final double maximumRotationVelocity;

    /**
     * A restriction that limits the maximum linear and rotational velocity.
     *
     * @param maximumTranslationVelocityMetersPerSecond maximum linear velocity
     * @param maximumRotationVelocityRadiansPerSecond   maximum rotational velocity
     */
    public VelocityRestrictedDrive(double maximumTranslationVelocityMetersPerSecond, double maximumRotationVelocityRadiansPerSecond) {
        this.maximumTranslationVelocity = MathUtil.clamp(maximumTranslationVelocityMetersPerSecond / SwerveConstants.MAXIMUM_SPEED_METERS_PER_SECOND, 0, 1);
        this.maximumRotationVelocity = MathUtil.clamp(maximumRotationVelocityRadiansPerSecond / SwerveConstants.MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND, 0, 1);
    }

    @Override
    public Translation2d applyRestrictionToTranslation(Translation2d targetTranslation) {
        final double translationMagnitude = targetTranslation.getNorm();
        if (translationMagnitude > maximumTranslationVelocity)
            return targetTranslation.times(maximumTranslationVelocity / translationMagnitude);
        return targetTranslation;
    }

    @Override
    public double applyRestrictionToRotation(double targetRotation) {
        return MathUtil.clamp(targetRotation, -maximumRotationVelocity, maximumRotationVelocity);
    }
}