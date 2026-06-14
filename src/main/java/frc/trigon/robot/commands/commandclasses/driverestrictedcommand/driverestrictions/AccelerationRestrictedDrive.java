package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A restriction that caps how the robot's linear and rotational acceleration.
 * Used for protecting top-heavy or extended mechanisms from tipping or damage caused by sudden joystick inputs.
 */
public class AccelerationRestrictedDrive implements DriveRestriction {
    private final double maximumTranslationAcceleration;
    private final SlewRateLimiter rotationLimiter;

    private Translation2d currentTranslation = Translation2d.kZero;

    private static final double LOOP_PERIOD_SECONDS = 0.02;

    /**
     * A restriction that limits the maximum linear and rotational acceleration.
     *
     * @param maximumTranslationAcceleration maximum linear acceleration
     * @param maximumRotationAcceleration    maximum rotational acceleration
     */
    public AccelerationRestrictedDrive(double maximumTranslationAcceleration, double maximumRotationAcceleration) {
        this.maximumTranslationAcceleration = maximumTranslationAcceleration;
        this.rotationLimiter = new SlewRateLimiter(maximumRotationAcceleration);
    }

    @Override
    public void init() {
        currentTranslation = Translation2d.kZero;
        rotationLimiter.reset(0);
    }

    @Override
    public Translation2d applyRestrictionToTranslation(Translation2d targetTranslation) {
        currentTranslation = MathUtil.slewRateLimit(
                currentTranslation,
                targetTranslation,
                LOOP_PERIOD_SECONDS,
                maximumTranslationAcceleration
        );
        return currentTranslation;
    }

    @Override
    public double applyRestrictionToRotation(double targetRotation) {
        return rotationLimiter.calculate(targetRotation);
    }
}