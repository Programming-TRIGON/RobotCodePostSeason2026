package frc.trigon.robot.commands.commandclasses.driverestrictedcommand.driverestrictions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.constants.RobotConstants;
import frc.trigon.robot.subsystems.swerve.SwerveConstants;

/**
 * A drive restriction that limits the robot's linear and rotational acceleration.
 */
public class AccelerationRestrictedDrive implements DriveRestriction {
    private final double maximumTranslationAccelerationMeter;
    private final SlewRateLimiter rotationAccelerationLimiter;
    private Translation2d currentTranslation = Translation2d.kZero;

    /**
     * A drive restriction that limits the maximum linear and rotational acceleration.
     *
     * @param maximumTranslationAccelerationMeter maximum linear acceleration
     * @param maximumRotationAcceleration    maximum rotational acceleration
     */
    public AccelerationRestrictedDrive(double maximumTranslationAccelerationMeter, double maximumRotationAcceleration) {
        this.maximumTranslationAccelerationMeter = maximumTranslationAccelerationMeter;
        this.rotationAccelerationLimiter = new SlewRateLimiter(maximumRotationAcceleration / SwerveConstants.MAXIMUM_ROTATIONAL_SPEED_RADIANS_PER_SECOND);
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
                maximumTranslationAccelerationMeter
        );
        return currentTranslation;
    }

    @Override
    public double applyRotationRestriction(double targetRotation) {
        return rotationAccelerationLimiter.calculate(targetRotation);
    }
}