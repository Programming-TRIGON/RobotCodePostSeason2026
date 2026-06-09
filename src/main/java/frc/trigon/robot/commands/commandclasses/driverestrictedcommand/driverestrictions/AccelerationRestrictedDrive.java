package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A restriction that caps how the robot's linear and rotational acceleration.
 * Used for protecting top-heavy or extended mechanisms from tipping or damage caused by sudden joystick inputs.
 */
public class AccelerationRestrictedDrive implements DriveRestriction {
    private final SlewRateLimiter translationLimiter;
    private final SlewRateLimiter rotationLimiter;

    /**
     * A restriction that limits the maximum linear and rotational acceleration.
     *
     * @param maximumTranslationAcceleration maximum linear acceleration
     * @param maximumRotationAcceleration    maximum rotational acceleration
     */
    public AccelerationRestrictedDrive(double maximumTranslationAcceleration, double maximumRotationAcceleration) {
        this.translationLimiter = new SlewRateLimiter(maximumTranslationAcceleration);
        this.rotationLimiter = new SlewRateLimiter(maximumRotationAcceleration);
    }

    @Override
    public void init() {
        translationLimiter.reset(0);
        rotationLimiter.reset(0);
    }

    @Override
    public Translation2d applyRestrictionToTranslation(Translation2d targetTranslation) {
        final double targetMagnitude = targetTranslation.getNorm();
        final double limitedMagnitude = translationLimiter.calculate(targetMagnitude);
        if (targetMagnitude == 0)
            return Translation2d.kZero;
        return targetTranslation.times(limitedMagnitude / targetMagnitude);
    }

    @Override
    public double applyRestrictionToRotation(double targetRotation) {
        return rotationLimiter.calculate(targetRotation);
    }
}