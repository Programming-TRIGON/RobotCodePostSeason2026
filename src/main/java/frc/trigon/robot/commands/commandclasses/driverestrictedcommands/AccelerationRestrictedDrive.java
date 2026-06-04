package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.commands.DriveRestriction;

/**
 * A restriction that limits the linear and rotational acceleration of the robot.
 */
public class AccelerationRestrictedDrive implements DriveRestriction {
    private final SlewRateLimiter translationLimiter;
    private final SlewRateLimiter thetaLimiter;

    /**
     * A restriction that limits the maximum linear and rotational acceleration.
     *
     * @param maximumTranslationAcceleration Maximum linear acceleration.
     * @param maximumThetaAcceleration       Maximum rotational velocity.
     */
    public AccelerationRestrictedDrive(double maximumTranslationAcceleration, double maximumThetaAcceleration) {
        this.translationLimiter = new SlewRateLimiter(maximumTranslationAcceleration);
        this.thetaLimiter = new SlewRateLimiter(maximumThetaAcceleration);
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
    public double applyRestrictionToTheta(double targetTheta) {
        return thetaLimiter.calculate(targetTheta);
    }
}