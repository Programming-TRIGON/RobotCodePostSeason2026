package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.constants.RobotConstants;

/**
 * A restriction that limits the robot's linear and rotational acceleration.
 */
public class AccelerationRestrictedDrive implements DriveRestriction {
    private final double maximumTranslationAcceleration;
    private final SlewRateLimiter rotationAccelerationLimiter;
    private Translation2d currentTranslation = Translation2d.kZero;

    /**
     * A restriction that limits the maximum linear and rotational acceleration.
     *
     * @param maximumTranslationAcceleration maximum linear acceleration
     * @param maximumRotationAcceleration    maximum rotational acceleration
     */
    public AccelerationRestrictedDrive(double maximumTranslationAcceleration, double maximumRotationAcceleration) {
        this.maximumTranslationAcceleration = maximumTranslationAcceleration;
        this.rotationAccelerationLimiter = new SlewRateLimiter(maximumRotationAcceleration);
    }

    @Override
    public void init() {
        currentTranslation = Translation2d.kZero;
        rotationAccelerationLimiter.reset(0);
    }

    @Override
    public Translation2d applyTranslationRestriction(Translation2d targetTranslation) {
        currentTranslation = MathUtil.slewRateLimit(
                currentTranslation,
                targetTranslation,
                RobotConstants.PERIODIC_TIME_SECONDS,
                maximumTranslationAcceleration
        );
        return currentTranslation;
    }

    @Override
    public double applyRotationRestriction(double targetRotation) {
        return rotationAccelerationLimiter.calculate(targetRotation);
    }
}