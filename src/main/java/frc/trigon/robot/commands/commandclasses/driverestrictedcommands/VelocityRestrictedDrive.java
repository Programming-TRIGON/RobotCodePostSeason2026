package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.commands.DriveRestriction;

/**
 * A restriction that limits the linear and rotational velocity of the robot.
 */
public class VelocityRestrictedDrive implements DriveRestriction {
    private final double maximumTranslationVelocity;
    private final double maximumThetaVelocity;

    /**
     * A restriction that limits the maximum linear and rotational velocity.
     *
     * @param maximumTranslationVelocity Maximum linear velocity, has to be between 0 and 1.
     * @param maximumThetaVelocity       Maximum rotational velocity, has to between 0 and 1.
     */
    public VelocityRestrictedDrive(double maximumTranslationVelocity, double maximumThetaVelocity) {
        this.maximumTranslationVelocity = MathUtil.clamp(maximumTranslationVelocity, 0, 1);
        this.maximumThetaVelocity = MathUtil.clamp(maximumThetaVelocity, 0, 1);
    }

    @Override
    public Translation2d applyRestrictionToTranslation(Translation2d targetTranslation) {
        final double norm = targetTranslation.getNorm();
        if (norm > maximumTranslationVelocity) {
            return targetTranslation.times(maximumTranslationVelocity / norm);
        }
        return targetTranslation;
    }

    @Override
    public double applyRestrictionToTheta(double targetTheta) {
        return MathUtil.clamp(targetTheta, -maximumThetaVelocity, maximumThetaVelocity);
    }
}