package frc.trigon.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;

public interface DriveRestriction {
    default Translation2d applyRestrictionToTranslation(Translation2d targetTranslation) {
        return targetTranslation;
    }

    default double applyRestrictionToTheta(double targetTheta) {
        return targetTheta;
    }
}