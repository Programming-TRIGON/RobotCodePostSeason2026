package frc.trigon.robot.commands.commandclasses.driverestrictedcommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.trigon.robot.commands.DriveRestriction;


public class VelocityRestrictedDrive implements DriveRestriction {
    private final double maximumTranslationVelocity;
    private final double maximumThetaVelocity;

    public VelocityRestrictedDrive(double maximumTranslationVelocity, double maximumThetaVelocity) {
        this.maximumTranslationVelocity = maximumTranslationVelocity;
        this.maximumThetaVelocity = maximumThetaVelocity;
    }

    @Override
    public Translation2d applyRestrictionToTranslation(Translation2d targetTranslation) {
        final double norm = targetTranslation.getNorm();
        if (norm > maximumTranslationVelocity) {
            return targetTranslation.times(maximumTranslationVelocity/norm);
        }
        return targetTranslation;
    }

    @Override
    public double applyRestrictionToTheta(double targetTheta) {
        return MathUtil.clamp(targetTheta, -maximumThetaVelocity, maximumThetaVelocity);
    }
}